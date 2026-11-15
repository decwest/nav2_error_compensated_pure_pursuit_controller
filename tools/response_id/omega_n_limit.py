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

"""Phase-margin guideline for the local ECPP loop with a measured actuator model."""

import argparse
import json
import math
from pathlib import Path


def validate_model(K, L, T, zeta=1.0, control_period=0.0):
    if not all(math.isfinite(x) for x in (K, L, T, zeta, control_period)):
        raise ValueError('model parameters must be finite')
    if K <= 0 or zeta <= 0 or min(L, T, control_period) < 0:
        raise ValueError('K and zeta must be positive; delays and period must be nonnegative')


def margins(omega_n, K, L, T, zeta=1.0, control_period=0.0):
    """Return crossover [rad/s] and unwrapped phase margin [degrees].

    L excludes sampling delay. Half the control period is added exactly once.
    The dimensionless crossover x = frequency / omega_n has monotone magnitude.
    """
    validate_model(K, L, T, zeta, control_period)
    if not math.isfinite(omega_n) or omega_n <= 0:
        raise ValueError('omega_n must be finite and positive')

    def log_magnitude(x):
        return (math.log(K) + math.log(math.hypot(1.0, 2 * zeta * x))
                - 2 * math.log(x) - math.log(math.hypot(1.0, T * omega_n * x)))

    lo = hi = 1.0
    while log_magnitude(lo) < 0:
        lo /= 2
    while log_magnitude(hi) > 0:
        hi *= 2
    for _ in range(80):
        mid = math.sqrt(lo * hi)
        if log_magnitude(mid) > 0:
            lo = mid
        else:
            hi = mid
    x = math.sqrt(lo * hi)
    wc = x * omega_n
    pm = math.degrees(math.atan(2 * zeta * x) - math.atan(T * wc)
                      - (L + control_period / 2) * wc)
    return wc, pm


def omega_n_limit(K, L, T, control_period, zeta=1.0, target_pm=30.0):
    """Return the first margin boundary, infinity if unbounded, or zero if infeasible."""
    validate_model(K, L, T, zeta, control_period)
    if not math.isfinite(target_pm) or not 0 < target_pm < 90:
        raise ValueError('target phase margin must be between 0 and 90 degrees')
    ideal_pm = margins(1.0, K, 0.0, 0.0, zeta)[1]
    if target_pm > ideal_pm:
        return 0.0
    if T == 0 and L + control_period / 2 == 0:
        return math.inf
    if target_pm == ideal_pm:
        return 0.0
    lo, hi = 0.0, 1.0
    for _ in range(1024):
        if margins(hi, K, L, T, zeta, control_period)[1] <= target_pm:
            break
        hi *= 2
    else:
        raise ValueError('could not bracket the phase-margin limit')
    for _ in range(80):
        mid = (lo + hi) / 2
        if margins(mid, K, L, T, zeta, control_period)[1] >= target_pm:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, help='JSON written by fit.py')
    parser.add_argument('--axis', choices=('angular', 'linear'), default='angular')
    parser.add_argument('--gain', type=float)
    parser.add_argument('--delay', type=float, help='dead time [s], excluding half a control period')
    parser.add_argument('--time-constant', type=float)
    parser.add_argument('--control-period', type=float, required=True, help='controller period [s]')
    parser.add_argument('--zeta', type=float, default=1.0)
    parser.add_argument('--phase-margin', type=float, default=30.0)
    parser.add_argument('--omega-n', type=float, required=True, help='candidate natural frequency [rad/s]')
    parser.add_argument('--speed', type=float, required=True, help='forward speed for PP lookahead [m/s]')
    args = parser.parse_args(argv)
    try:
        explicit = (args.gain, args.delay, args.time_constant)
        if args.model:
            if any(x is not None for x in explicit):
                raise ValueError('use either --model or all three explicit model parameters')
            data = json.loads(args.model.read_text())
            if data.get('schema_version') != 1 or data.get('sampling_delay_included') is not False:
                raise ValueError('expected fit.py schema version 1 without sampling delay')
            model = data['axes'][args.axis]['recommended']
            K, L, T = (model[k] for k in ('K', 'L', 'T'))
        else:
            if any(x is None for x in explicit):
                raise ValueError('provide --model or --gain, --delay and --time-constant')
            K, L, T = explicit
        if not math.isfinite(args.speed) or args.speed < 0:
            raise ValueError('speed must be finite and nonnegative')
        cap = omega_n_limit(K, L, T, args.control_period, args.zeta, args.phase_margin)
        wc, pm = margins(args.omega_n, K, L, T, args.zeta, args.control_period)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    print(f'Model: K={K:.6g}, L={L:.6g} s, T={T:.6g} s')
    print(f'Effective dead time: {L + args.control_period / 2:.6g} s (includes half a control period)')
    print(f'omega_n limit at {args.phase_margin:g} deg: {cap:.6g} rad/s')
    print(f'Candidate {args.omega_n:g} rad/s: phase margin {pm:.3f} deg; crossover {wc:.6g} rad/s')
    print(f'Equivalent PP lookahead: {math.sqrt(2) * args.speed / args.omega_n:.6g} m')
    print('Candidate meets target.' if pm >= args.phase_margin else 'Candidate is below the target phase margin.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
