#!/usr/bin/env python3
# Copyright (c) 2026 Fumiya Ohnishi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Fit forward/yaw responses K exp(-L s)/(T s + 1) from CSV or rosbag2.

CSV columns: t, v_command, w_command, v_odom, w_odom, and optionally
v_body_command, w_body_command. Units are seconds, m/s and rad/s.
Legacy *_nav and *_fin columns are accepted as aliases. Bag timestamps are
receipt times; commands use zero-order hold and odometry linear interpolation.
"""

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np


def zoh(grid, times, values):
    return values[np.clip(np.searchsorted(times, grid, side='right') - 1, 0, len(values) - 1)]


def simulate(u, delay, time_constant, dt):
    """Backward-Euler first-order filter; delay is rounded to the nearest sample.

    The input at each sample drives that sample's filtered output, matching the
    reference identification convention. Initial output is the initial input
    (steady prehistory); recordings should include rest before excitation.
    """
    k = int(round(delay / dt))
    delayed = np.asarray(u)[np.maximum(np.arange(len(u)) - k, 0)]
    if time_constant == 0:
        return delayed.copy()
    a = time_constant / (time_constant + dt)
    y = np.empty_like(delayed, dtype=float)
    y[0] = delayed[0]
    for i in range(1, len(y)):
        y[i] = a * y[i - 1] + (1 - a) * delayed[i]
    return y


def fit_fopdt(runs, dt=0.005, max_delay=0.4, max_time_constant=0.6, time_step=0.01):
    """Joint least squares over independent runs; K is solved analytically.

    L and T use time_step, with L rounded to the sampling grid. All runs share K, L, T,
    but retain separate initial conditions. No DC offset is fitted.
    """
    if not all(math.isfinite(x) and x > 0 for x in (dt, time_step, max_time_constant)):
        raise ValueError('sampling interval, T step and maximum T must be positive')
    if not math.isfinite(max_delay) or max_delay < 0:
        raise ValueError('maximum delay must be finite and nonnegative')
    if not runs:
        raise ValueError('no runs to fit')
    for u, y in runs:
        if len(u) != len(y) or len(u) < 20 or not np.isfinite([u, y]).all():
            raise ValueError('each run needs at least 20 paired, finite samples')
        if np.ptp(u) < 1e-6 or np.ptp(y) < 1e-6:
            raise ValueError('insufficient excitation or constant measured response')
    max_k = int(math.floor(max_delay / dt + 1e-9))
    if any(max_k >= len(u) - 10 for u, _ in runs):
        raise ValueError('recording is too short for the requested maximum delay')
    taus = np.arange(int(max_time_constant / time_step + 1e-9) + 1) * time_step
    alpha = np.zeros_like(taus)
    alpha[1:] = taus[1:] / (taus[1:] + dt)
    filtered = []
    for u, y in runs:
        bank = np.empty((len(taus), len(u)))
        bank[:, 0] = u[0]
        for i in range(1, len(u)):
            bank[:, i] = alpha * bank[:, i - 1] + (1 - alpha) * u[i]
        filtered.append((bank, y))
    yy = sum(float(y @ y) for _, y in runs)
    sst = sum(float((y - y.mean()) @ (y - y.mean())) for _, y in runs)
    best = None
    for k in sorted(set(round(x / dt) for x in np.arange(0, max_delay + time_step * 1e-6, time_step))):
        cross = np.zeros(len(taus))
        energy = np.zeros(len(taus))
        for bank, y in filtered:
            delayed = bank if k == 0 else np.concatenate((np.repeat(bank[:, :1], k, axis=1), bank[:, :-k]), axis=1)
            cross += np.einsum('ij,j->i', delayed, y)
            energy += np.einsum('ij,ij->i', delayed, delayed)
        gains = np.divide(cross, energy, out=np.zeros_like(cross), where=energy > 1e-12)
        errors = np.where((energy > 1e-12) & (gains > 0), yy - 2 * gains * cross + gains**2 * energy, np.inf)
        j = int(np.argmin(errors))
        if best is None or errors[j] < best[0]:
            best = (float(errors[j]), float(gains[j]), k * dt, float(taus[j]))
    error, gain, delay, tau = best
    if not math.isfinite(error):
        raise ValueError('no positive-gain model fits this signal')
    error = max(error, 0.0)
    return dict(K=gain, L=delay, T=tau, r2=1 - error / sst,
                rmse=math.sqrt(error / sum(len(y) for _, y in runs)),
                at_search_boundary=bool(delay >= max_k * dt and max_k > 0 or tau == taus[-1]))


def summarize_fits(runs, dt, max_delay, max_time_constant, time_step):
    """Median model, retaining the median L+T because these terms trade off."""
    fits = [fit_fopdt([run], dt, max_delay, max_time_constant, time_step) for run in runs]
    K = float(np.median([m['K'] for m in fits]))
    T = float(np.median([m['T'] for m in fits]))
    L = max(float(np.median([m['L'] + m['T'] for m in fits])) - T, 0.0)
    error = sum(float(np.sum((y - K * simulate(u, L, T, dt))**2)) for u, y in runs)
    sst = sum(float(np.sum((y - y.mean())**2)) for _, y in runs)
    return dict(K=K, L=L, T=T, r2=1 - error / sst,
                rmse=math.sqrt(error / sum(len(y) for _, y in runs)),
                per_run=fits, at_search_boundary=any(m['at_search_boundary'] for m in fits))


def xcorr_delay(u, y, dt, max_delay):
    """Nonnegative pipeline delay from demeaned normalized cross-correlation."""
    best = (-math.inf, 0)
    for k in range(min(int(max_delay / dt + 1e-9), len(u) - 10) + 1):
        a, b = (u[:-k], y[k:]) if k else (u, y)
        a, b = a - a.mean(), b - b.mean()
        den = np.linalg.norm(a) * np.linalg.norm(b)
        score = float(a @ b / den) if den > 1e-12 else -math.inf
        if score > best[0]:
            best = (score, k)
    return dict(L=best[1] * dt, correlation=best[0])


def load_csv(path, dt):
    raw = np.genfromtxt(path, delimiter=',', names=True, encoding='utf-8-sig')
    names = raw.dtype.names or ()
    aliases = {f'{a}_{b}': f'{a}_{c}' for a in ('v', 'w')
               for b, c in (('nav', 'command'), ('fin', 'body_command'))}
    data = {aliases.get(name, name): np.atleast_1d(raw[name]) for name in names}
    required = ('t', 'v_command', 'w_command', 'v_odom', 'w_odom')
    if any(name not in data for name in required):
        raise ValueError(f'{path.name}: missing CSV columns; need {", ".join(required)}')
    if ('v_body_command' in data) != ('w_body_command' in data):
        raise ValueError('both body command columns are required when either is present')
    t = data['t']
    if len(t) < 20 or not all(np.isfinite(x).all() for x in data.values()) or np.any(np.diff(t) <= 0):
        raise ValueError('CSV must have finite values and at least 20 strictly increasing timestamps')
    grid = np.arange(0, t[-1] - t[0] + dt * 1e-6, dt)
    if len(grid) < 20:
        raise ValueError('CSV duration is too short for the sampling interval')
    # Do not interpolate already uniform exported series a second time.
    if len(grid) == len(t) and np.allclose(np.diff(t), dt, rtol=1e-3, atol=dt * 1e-6):
        return {k: (v - t[0] if k == 't' else v.copy()) for k, v in data.items()}
    out = {'t': grid}
    for name in required[1:] + (('v_body_command', 'w_body_command') if 'v_body_command' in data else ()):
        out[name] = (np.interp(grid, t - t[0], data[name]) if name.endswith('odom')
                     else zoh(grid, t - t[0], data[name]))
    return out


def load_bag(path, command_topic, body_topic, odom_topic, dt):
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise ValueError('bag input needs the optional rosbags dependency') from exc
    if not command_topic or not odom_topic:
        raise ValueError('bag input requires --command-topic and --odom-topic')
    selected = [command_topic, odom_topic] + ([body_topic] if body_topic else [])
    if len(set(selected)) != len(selected):
        raise ValueError('command, body-command and odom topics must be distinct')
    samples = {topic: [] for topic in selected}
    with AnyReader([path], default_typestore=get_typestore(Stores.ROS2_JAZZY)) as reader:
        connections = [c for c in reader.connections if c.topic in samples]
        for connection, timestamp, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            if connection.topic == odom_topic:
                if connection.msgtype != 'nav_msgs/msg/Odometry':
                    raise ValueError('odometry topic must contain nav_msgs/msg/Odometry')
                twist = msg.twist.twist
            else:
                if connection.msgtype not in ('geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped'):
                    raise ValueError('command topics must contain Twist or TwistStamped')
                twist = msg.twist if hasattr(msg, 'twist') else msg
            samples[connection.topic].append((timestamp, twist.linear.x, twist.angular.z))
    if any(len(values) < 2 for values in samples.values()):
        raise ValueError('a selected topic is missing or has fewer than two messages')
    # Subtract integer nanoseconds before converting to seconds.
    origin = min(values[0][0] for values in samples.values())
    arrays = {}
    for topic, values in samples.items():
        ordered = sorted({stamp: (v, w) for stamp, v, w in values}.items())
        arrays[topic] = np.array([((stamp - origin) * 1e-9, v, w) for stamp, (v, w) in ordered])
        if not np.isfinite(arrays[topic]).all():
            raise ValueError('bag contains nonfinite velocity data')
    cmd = arrays[command_topic]
    moving = np.flatnonzero(np.abs(cmd[:, 1:]).sum(axis=1) > 1e-3)
    if len(moving) < 2:
        raise ValueError('no command excitation in bag')
    start = max(cmd[moving[0], 0] - 0.5, *(x[0, 0] for x in arrays.values()))
    end = min(cmd[moving[-1], 0] + 0.5, *(x[-1, 0] for x in arrays.values()))
    grid = np.arange(start, end, dt)
    if len(grid) < 20:
        raise ValueError('selected topics have insufficient overlapping data')
    out = {'t': grid - grid[0]}
    for role, topic in [('command', command_topic), ('odom', odom_topic)] + ([('body_command', body_topic)] if body_topic else []):
        a = arrays[topic]
        for axis, col in (('v', 1), ('w', 2)):
            out[f'{axis}_{role}'] = (np.interp(grid, a[:, 0], a[:, col]) if role == 'odom'
                                    else zoh(grid, a[:, 0], a[:, col]))
    return out


def analyze(series, dt=0.005, max_delay=0.4, max_time_constant=0.6, time_step=0.01):
    has_body = ['w_body_command' in s for s in series]
    if any(has_body) and not all(has_body):
        raise ValueError('all inputs must consistently include or omit body command columns')
    result = dict(schema_version=1, sampling_delay_included=False, dt=dt,
                  method='per-run backward-Euler grid search, median K/T/L+T, steady initial state',
                  search=dict(max_delay=max_delay, max_time_constant=max_time_constant, time_step=time_step),
                  axes={}, warnings=[])
    for axis, prefix in (('angular', 'w'), ('linear', 'v')):
        runs = [(s[f'{prefix}_command'], s[f'{prefix}_odom']) for s in series]
        if any(np.ptp(u) < 1e-6 or np.ptp(y) < 1e-6 for u, y in runs):
            result['axes'][axis] = {'status': 'insufficient excitation or constant measured response'}
            result['warnings'].append(f'{axis}: no usable response model')
            continue
        overall = summarize_fits(runs, dt, max_delay, max_time_constant, time_step)
        models = {'overall': overall, 'recommended': dict(overall, source='overall')}
        if all(has_body):
            body = summarize_fits([(s[f'{prefix}_body_command'], s[f'{prefix}_odom']) for s in series],
                             dt, max_delay, max_time_constant, time_step)
            pipeline = [xcorr_delay(s[f'{prefix}_command'], s[f'{prefix}_body_command'], dt, max_delay) for s in series]
            delay = float(np.median([p['L'] for p in pipeline]))
            models.update(body=body, pipeline=dict(L=delay, per_run=pipeline, assumed_gain=1.0),
                          recommended=dict(K=body['K'], L=delay + body['L'], T=body['T'],
                                           source='pipeline delay + body', r2_body=body['r2']))
            result['warnings'].append(f'{axis}: recommended cascade assumes the pipeline has unit gain and pure delay; inspect overall fit and time series')
        for stage in ('overall', 'body'):
            if stage in models:
                model = models[stage]
                if model['at_search_boundary']:
                    result['warnings'].append(f'{axis}/{stage}: optimum touches search bounds; expand them')
                if model['r2'] < 0.9:
                    result['warnings'].append(f'{axis}/{stage}: R2 below 0.9; inspect fit before selecting a limit')
        result['axes'][axis] = models
    if not any('recommended' in a for a in result['axes'].values()):
        raise ValueError('no axis has sufficient excitation and measured response')
    return result


def write_series(path, data):
    names = list(data)
    np.savetxt(path, np.column_stack([data[k] for k in names]), delimiter=',', header=','.join(names), comments='', fmt='%.9g')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', nargs='+', type=Path, help='CSV files or rosbag2 directories; median of independently fitted runs')
    parser.add_argument('--command-topic')
    parser.add_argument('--body-command-topic')
    parser.add_argument('--odom-topic')
    parser.add_argument('--output', type=Path, required=True, help='new output directory')
    parser.add_argument('--dt', type=float, default=0.005)
    parser.add_argument('--max-delay', type=float, default=0.4)
    parser.add_argument('--max-time-constant', type=float, default=0.6)
    parser.add_argument('--time-step', type=float, default=0.01)
    parser.add_argument('--plot', action='store_true', help='write fit.png (requires matplotlib)')
    args = parser.parse_args(argv)
    try:
        if not math.isfinite(args.dt) or args.dt <= 0:
            raise ValueError('--dt must be finite and positive')
        if args.output.exists():
            raise ValueError('output directory already exists; choose a new path')
        plt = None
        if args.plot:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        series = [(load_csv(p, args.dt) if p.suffix.lower() == '.csv' else
                   load_bag(p, args.command_topic, args.body_command_topic, args.odom_topic, args.dt)) for p in args.inputs]
        result = analyze(series, args.dt, args.max_delay, args.max_time_constant, args.time_step)
        result['inputs'] = [p.name for p in args.inputs]
        args.output.mkdir(parents=True)
        (args.output / 'model.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        print('Axis     Stage          K       L [s]    T [s]    R2')
        for axis, models in result['axes'].items():
            for stage in ('body', 'overall', 'recommended'):
                if stage in models:
                    m = models[stage]
                    r2 = f"{m['r2']:.5f}" if 'r2' in m else 'see body'
                    print(f"{axis:8} {stage:12} {m['K']:7.4f} {m['L']:8.4f} {m['T']:8.4f} {r2}")
        if plt:
            fig, axes = plt.subplots(2, 1, figsize=(10, 7))
        for index, data in enumerate(series, 1):
            for j, (axis, prefix) in enumerate((('angular', 'w'), ('linear', 'v'))):
                models = result['axes'][axis]
                for stage, source in (('body', 'body_command'), ('overall', 'command')):
                    if stage in models:
                        m = models[stage]
                        data[f'{prefix}_{stage}_fit'] = m['K'] * simulate(data[f'{prefix}_{source}'], m['L'], m['T'], args.dt)
                if plt:
                    for suffix in ('command', 'body_command', 'odom', 'body_fit', 'overall_fit'):
                        name = f'{prefix}_{suffix}'
                        if name in data:
                            axes[j].plot(data['t'], data[name], label=f'{index}: {suffix}', linewidth=1)
                    axes[j].set_ylabel('Yaw rate [rad/s]' if j == 0 else 'Forward speed [m/s]')
            write_series(args.output / f'series_{index:02d}.csv', data)
        if plt:
            for ax in axes:
                ax.legend(fontsize='small')
                ax.grid(True)
            axes[-1].set_xlabel('Time [s]')
            fig.tight_layout()
            fig.savefig(args.output / 'fit.png', dpi=150)
            plt.close(fig)
        for warning in result['warnings']:
            print(f'NOTE: {warning}', file=sys.stderr)
    except (ValueError, OSError, ImportError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
