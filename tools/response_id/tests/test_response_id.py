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

"""ROS-free identification, phase-margin and command safety regressions."""

import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def load_tool(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


excite = load_tool('excite')
fit = load_tool('fit')
limit = load_tool('omega_n_limit')


def arguments(mode='drive', **overrides):
    args = excite.make_parser().parse_args(
        [
            '--topic',
            '/test/cmd',
            '--message-type',
            'Twist',
            '--mode',
            mode,
            '--speed',
            '0.2',
            '--amplitude',
            '0.1',
            '--linear-accel',
            '0.3',
            '--angular-accel',
            '0.8',
            '--period',
            '0.02',
            '--footprint-radius',
            '0.4',
            '--clearance',
            '0.2',
        ]
    )
    if mode == 'spin':
        args.speed = 0.0
    if mode == 'straight':
        args.amplitude = 0.0
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


@pytest.mark.parametrize('mode', ['spin', 'drive', 'straight', 'sine'])
@pytest.mark.parametrize('period', [0.01, 0.0333333333333333, 0.1])
def test_command_limits_and_final_stop(mode, period):
    args = arguments(mode, period=period)
    sequence = np.array(excite.build_sequence(args))
    assert sequence[0, 1:].tolist() == [0, 0]
    assert np.max(np.abs(np.diff(sequence[:, 1]) / period)) <= args.linear_accel + 1e-10
    assert np.max(np.abs(np.diff(sequence[:, 2]) / period)) <= args.angular_accel + 1e-10
    assert np.max(sequence[:, 1]) <= args.speed + 1e-10
    assert np.max(np.abs(sequence[:, 2])) <= args.amplitude + 1e-10
    assert np.all(sequence[-math.ceil(1 / period):, 1:] == 0)
    assert sequence[-1, 0] + args.countdown + excite.STARTUP_SECONDS + 1 <= args.max_duration
    if mode == 'spin':
        assert np.all(sequence[:, 1] == 0)
    if mode == 'straight':
        assert np.all(sequence[:, 2] == 0)


@pytest.mark.parametrize(
    'overrides',
    [
        {'linear_accel': 0},
        {'speed': float('nan')},
        {'angular_accel': -1},
        {'period': float('inf')},
        {'max_duration': 5},
        {'countdown': 0},
        {'repeats': 0},
        {'footprint_radius': 0},
        {'clearance': -1},
        {'record': Path('example'), 'record_topics': ['/test/odom']},
    ],
)
def test_invalid_motion_is_rejected(overrides):
    with pytest.raises(ValueError):
        excite.build_sequence(arguments(**overrides))


def test_sine_envelope_acceleration_is_validated():
    # The bare cosine fits; its envelope derivative exceeds the available limit.
    with pytest.raises(ValueError, match='envelope'):
        excite.build_sequence(arguments('sine', amplitude=0.1, frequency=1, angular_accel=0.65))


def test_space_estimate_includes_spin_footprint():
    args = arguments('spin')
    estimate = excite.motion_estimate(excite.build_sequence(args), 0.4, 0.2)
    assert estimate['ideal_travel_m'] == 0
    assert estimate['ideal_clear_width_m'] == pytest.approx(1.2)
    assert estimate['ideal_clear_length_m'] == pytest.approx(1.2)


def test_dry_run_emits_csv_without_ros():
    completed = subprocess.run(
        [
            sys.executable,
            str(ROOT / 'excite.py'),
            '--topic',
            '/test/cmd',
            '--message-type',
            'TwistStamped',
            '--mode',
            'spin',
            '--speed',
            '0',
            '--amplitude',
            '0.2',
            '--linear-accel',
            '0.4',
            '--angular-accel',
            '0.6',
            '--period',
            '0.02',
            '--footprint-radius',
            '0.4',
            '--clearance',
            '0.2',
            '--dry-run',
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    assert completed.stdout.startswith('t,v,w\n')
    assert 'Ideal command-following space' in completed.stderr
    assert completed.stdout.strip().splitlines()[-1].endswith(',0.0,0.0')


@pytest.mark.parametrize('gain,delay,tau', [(0.85, 0.04, 0.12), (1.2, 0.08, 0.25)])
def test_recover_independent_continuous_step_response(gain, delay, tau):
    dt = 0.005
    t = np.arange(0, 8, dt)
    u, y = np.zeros_like(t), np.zeros_like(t)
    # Exact continuous response to a sequence of steps, independent of the fitter.
    for time, change in [(0.5, 0.7), (2.0, -1.2), (4.0, 0.8), (6.0, -0.3)]:
        u += change * (t >= time)
        elapsed = np.maximum(0, t - time - delay)
        y += gain * change * (1 - np.exp(-elapsed / tau))
    y += np.random.default_rng(123).normal(0, 0.001, len(y))
    model = fit.fit_fopdt([(u, y)], dt=dt, time_step=0.005)
    assert model['K'] == pytest.approx(gain, abs=0.01)
    assert model['L'] == pytest.approx(delay, abs=0.01)
    assert model['T'] == pytest.approx(tau, abs=0.01)
    assert model['r2'] > 0.999


def test_uniform_timestamp_rounding_does_not_shift_held_commands(tmp_path):
    # A grid built at Unix epoch has small per-step rounding. It is already sampled.
    t = np.arange(1000) * 0.005000114440917969
    u = np.where(np.arange(1000) % 100 < 50, 0.0, 0.5)
    path = tmp_path / 'series.csv'
    fit.write_series(path, {'t': t, 'v_command': u, 'w_command': u, 'v_odom': u, 'w_odom': u})
    data = fit.load_csv(path, 0.005)
    np.testing.assert_array_equal(data['w_command'], u)


@pytest.mark.parametrize('kind', ['duplicate', 'nonfinite', 'missing'])
def test_invalid_csv_is_rejected(tmp_path, kind):
    t = np.arange(100) * 0.005
    u = np.sin(t)
    data = {'t': t, 'v_command': u, 'w_command': u, 'v_odom': u, 'w_odom': u}
    if kind == 'duplicate':
        data['t'][1] = data['t'][0]
    elif kind == 'nonfinite':
        data['w_command'][1] = np.nan
    else:
        del data['w_command']
    path = tmp_path / 'invalid.csv'
    fit.write_series(path, data)
    with pytest.raises(ValueError):
        fit.load_csv(path, 0.005)


def test_no_excitation_is_rejected():
    with pytest.raises(ValueError, match='excitation'):
        fit.fit_fopdt([(np.ones(500), np.ones(500))])


@pytest.mark.parametrize('gain,zeta,delay,period', [(1, 1, 0.08, 0), (0.8, 0.7, 0.03, 0.04)])
def test_pure_delay_limit_matches_closed_form(gain, zeta, delay, period):
    # With T=0, x=wc/wn solves x^4 - 4 K^2 zeta^2 x^2 - K^2 = 0.
    x = math.sqrt(2 * gain**2 * zeta**2 + math.sqrt(4 * gain**4 * zeta**4 + gain**2))
    ideal_phase = math.atan(2 * zeta * x)
    expected = (ideal_phase - math.radians(30)) / ((delay + period / 2) * x)
    cap = limit.omega_n_limit(gain, delay, 0, period, zeta)
    assert cap == pytest.approx(expected, rel=1e-10)
    assert limit.margins(cap, gain, delay, 0, zeta, period)[1] == pytest.approx(30)


def test_ideal_and_unattainable_limits():
    assert math.isinf(limit.omega_n_limit(1, 0, 0, 0))
    assert limit.omega_n_limit(1, 0, 0, 0, target_pm=89) == 0
    assert limit.margins(2, 1, 0, 0)[1] == pytest.approx(76.345415254, abs=1e-8)


def test_sampling_delay_is_added_once():
    assert limit.margins(2, 0.9, 0.04, 0.1, control_period=0.04) == pytest.approx(
        limit.margins(2, 0.9, 0.06, 0.1)
    )


@pytest.mark.parametrize('params', [(-1, 0, 0), (1, -1, 0), (1, 0, -1), (float('nan'), 0, 0)])
def test_invalid_models_are_rejected(params):
    with pytest.raises(ValueError):
        limit.omega_n_limit(*params, control_period=0.02)


@pytest.mark.parametrize(
    'filename,gain,delay,tau,cap,pm',
    [
        ('unoccupied.csv', 0.9281, 0.03, 0.06, 3.062, 44.9),
        ('occupied.csv', 0.8258, 0.00, 0.17, 2.297, 34.4),
    ],
)
def test_measured_recording_regression(filename, gain, delay, tau, cap, pm):
    result = fit.analyze([fit.load_csv(ROOT / 'tests' / 'data' / filename, 0.005)])
    body = result['axes']['angular']['body']
    assert body['K'] == pytest.approx(gain, abs=0.02)
    assert body['L'] + body['T'] == pytest.approx(delay + tau, abs=0.01)
    assert body['r2'] > 0.98
    model = result['axes']['angular']['recommended']
    values = [model[k] for k in ('K', 'L', 'T')]
    assert limit.omega_n_limit(*values, control_period=1 / 30) == pytest.approx(cap, abs=0.1)
    assert limit.margins(2, *values, control_period=1 / 30)[1] == pytest.approx(pm, abs=2)
    assert result['axes']['linear']['body']['K'] == pytest.approx(1, abs=0.03)


def test_csv_to_json_to_limit_cli(tmp_path, capsys):
    out = tmp_path / 'fit'
    assert fit.main([str(ROOT / 'tests/data/unoccupied.csv'), '--output', str(out)]) == 0
    model = json.loads((out / 'model.json').read_text())
    assert model['sampling_delay_included'] is False
    assert 'w_body_fit' in fit.load_csv(out / 'series_01.csv', 0.005)
    assert (
        limit.main(
            [
                '--model',
                str(out / 'model.json'),
                '--control-period',
                str(1 / 30),
                '--omega-n',
                '2',
                '--speed',
                '0.5',
            ]
        )
        == 0
    )
    assert 'Equivalent PP lookahead: 0.353553 m' in capsys.readouterr().out
    with pytest.raises(SystemExit):
        fit.main([str(ROOT / 'tests/data/unoccupied.csv'), '--output', str(out)])


@pytest.mark.parametrize('stamped', [False, True])
def test_ros_free_bag_reader(tmp_path, stamped):
    pytest.importorskip('rosbags')
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import get_typestore, Stores

    store = get_typestore(Stores.ROS2_JAZZY)
    types = store.types
    vector = types['geometry_msgs/msg/Vector3']
    twist_type = types['geometry_msgs/msg/Twist']
    time_type = types['builtin_interfaces/msg/Time']
    header_type = types['std_msgs/msg/Header']
    pose = types['geometry_msgs/msg/PoseWithCovariance'](
        types['geometry_msgs/msg/Pose'](
            types['geometry_msgs/msg/Point'](0.0, 0.0, 0.0),
            types['geometry_msgs/msg/Quaternion'](0.0, 0.0, 0.0, 1.0),
        ),
        np.zeros(36),
    )
    bag = tmp_path / 'bag'
    name = 'geometry_msgs/msg/TwistStamped' if stamped else 'geometry_msgs/msg/Twist'
    with Writer(bag, version=9) as writer:
        command = writer.add_connection('/test/cmd', name, typestore=store)
        odometry = writer.add_connection('/test/odom', 'nav_msgs/msg/Odometry', typestore=store)
        for i in range(400):
            stamp = 1700000000000000000 + i * 10000000
            header = header_type(time_type(stamp // 10**9, stamp % 10**9), 'base_link')
            value = 0.2 if 50 <= i < 250 else 0.0
            twist = twist_type(vector(value, 0.0, 0.0), vector(0.0, 0.0, value))
            msg = types[name](header, twist) if stamped else twist
            odom = types['nav_msgs/msg/Odometry'](
                header,
                'base_link',
                pose,
                types['geometry_msgs/msg/TwistWithCovariance'](twist, np.zeros(36)),
            )
            writer.write(command, stamp, store.serialize_cdr(msg, name))
            writer.write(odometry, stamp, store.serialize_cdr(odom, odometry.msgtype))
    series = fit.load_bag(bag, '/test/cmd', None, '/test/odom', 0.005)
    assert len(series['t']) >= 400
    assert np.max(series['w_command']) == pytest.approx(0.2)
    result = fit.analyze([series])
    assert result['axes']['angular']['recommended']['source'] == 'overall'
    assert result['axes']['angular']['overall']['K'] == pytest.approx(1, abs=0.01)
    with pytest.raises(ValueError, match='missing'):
        fit.load_bag(bag, '/missing', None, '/test/odom', 0.005)
