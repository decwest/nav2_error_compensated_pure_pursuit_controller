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

"""Generate bounded open-loop velocity commands, optionally publish and record them.

All robot-specific command values, limits and topics are explicit arguments.
--dry-run writes t,v,w CSV to stdout without importing ROS; estimates go to stderr.
Lateral and all other unused Twist components are always zero.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time


STARTUP_SECONDS = 5.0  # Discovery (2 s) and recorder subscription (up to 3 s).
STOP_SECONDS = 1.0


def validate(args):
    positive = ('linear_accel', 'angular_accel', 'period', 'hold', 'rest', 'max_duration', 'footprint_radius')
    nonnegative = ('speed', 'amplitude', 'clearance')
    for name in positive + nonnegative:
        value = getattr(args, name)
        if not math.isfinite(value) or (value <= 0 if name in positive else value < 0):
            raise ValueError(f'--{name.replace("_", "-")} must be finite and {"positive" if name in positive else "nonnegative"}')
    if not 0.001 <= args.period <= 0.1:
        raise ValueError('--period must be between 0.001 and 0.1 seconds')
    if not math.isfinite(args.countdown) or args.countdown < 1:
        raise ValueError('--countdown must be at least one second')
    if args.repeats < 1:
        raise ValueError('--repeats must be positive')
    if args.mode == 'spin' and (args.speed != 0 or args.amplitude <= 0):
        raise ValueError('spin requires zero speed and positive amplitude')
    if args.mode == 'straight' and (args.speed <= 0 or args.amplitude != 0):
        raise ValueError('straight requires positive speed and zero amplitude')
    if args.mode in ('drive', 'sine') and (args.speed <= 0 or args.amplitude <= 0):
        raise ValueError('drive and sine require positive speed and amplitude')
    if args.mode == 'sine':
        if not math.isfinite(args.frequency) or args.frequency <= 0 or args.cycles < 4:
            raise ValueError('sine needs positive frequency and at least four cycles')
        if args.period * args.frequency > 0.05:
            raise ValueError('sine requires at least 20 samples per cycle')
        # Envelope derivative contributes at most A*f during the one-cycle fall.
        if args.amplitude * args.frequency * (2 * math.pi + 1) > args.angular_accel:
            raise ValueError('sine including its envelope exceeds --angular-accel')
    if args.record and not args.record_topics:
        raise ValueError('--record requires an explicit --record-topics list')
    if args.record_topics and not args.record:
        raise ValueError('--record-topics requires --record')
    if args.record and args.topic not in args.record_topics:
        raise ValueError('--record-topics must include the command topic')
    if not args.topic.startswith('/'):
        raise ValueError('--topic must be an absolute ROS topic name')


def build_sequence(args):
    """Return (time, forward velocity, yaw rate), including ramps and final zeros."""
    validate(args)
    dt = args.period
    budget = args.max_duration - STARTUP_SECONDS - args.countdown - STOP_SECONDS - dt
    if budget <= STOP_SECONDS:
        raise ValueError('--max-duration leaves no time for motion after startup and countdown')
    samples = [(0.0, 0.0, 0.0)]

    def append(v, w):
        t = len(samples) * dt
        if t > budget + 1e-9:
            raise ValueError('sequence including startup, countdown and stopping exceeds --max-duration')
        samples.append((t, v, w))

    def segment(v_target, w_target, duration):
        for _ in range(math.ceil(duration / dt - 1e-10)):
            _, v, w = samples[-1]
            v += max(-args.linear_accel * dt, min(args.linear_accel * dt, v_target - v))
            w += max(-args.angular_accel * dt, min(args.angular_accel * dt, w_target - w))
            append(v, w)

    segment(0.0, 0.0, args.rest)
    if args.mode == 'spin':
        for _ in range(args.repeats):
            for direction in (1, -1):
                segment(0.0, direction * args.amplitude, args.hold + args.amplitude / args.angular_accel)
                segment(0.0, 0.0, args.rest + args.amplitude / args.angular_accel)
    elif args.mode == 'straight':
        for fraction in (1.0, 0.5, 1.0):
            segment(fraction * args.speed, 0.0, args.hold + args.speed / args.linear_accel)
    else:
        segment(args.speed, 0.0, args.hold + args.speed / args.linear_accel)
        if args.mode == 'drive':
            for _ in range(args.repeats):
                for multiplier, duration in ((1, args.hold), (-1, 2 * args.hold), (1, args.hold), (0, args.rest)):
                    segment(args.speed, multiplier * args.amplitude, duration)
        else:
            duration = args.cycles / args.frequency
            for i in range(1, math.ceil(duration / dt) + 1):
                t = min(i * dt, duration)
                envelope = max(0.0, min(1.0, t * args.frequency / 2, (duration - t) * args.frequency))
                w = envelope * args.amplitude * math.cos(2 * math.pi * args.frequency * t)
                append(args.speed, w)
    _, v, w = samples[-1]
    segment(0.0, 0.0, max(v / args.linear_accel, abs(w) / args.angular_accel) + dt)
    segment(0.0, 0.0, STOP_SECONDS)
    return samples


def motion_estimate(samples, footprint_radius, clearance):
    """Ideal unicycle center envelope, inflated by footprint radius and clearance.

    Arc length is also reported as a heading-independent displacement bound.
    This assumes perfect command following and is not a measured stopping bound.
    """
    x = y = heading = travel = 0.0
    xs, ys = [0.0], [0.0]
    for (t, v, w), (next_t, _, _) in zip(samples, samples[1:]):
        dt = next_t - t
        if abs(w) > 1e-12:
            x += v / w * (math.sin(heading + w * dt) - math.sin(heading))
            y += v / w * (math.cos(heading) - math.cos(heading + w * dt))
        else:
            x += v * dt * math.cos(heading)
            y += v * dt * math.sin(heading)
        heading += w * dt
        travel += abs(v) * dt
        xs.append(x)
        ys.append(y)
    inflation = footprint_radius + clearance
    return dict(command_duration_s=samples[-1][0], ideal_travel_m=travel,
                ideal_x_min_m=min(xs) - inflation, ideal_x_max_m=max(xs) + inflation,
                ideal_y_min_m=min(ys) - inflation, ideal_y_max_m=max(ys) + inflation,
                ideal_clear_length_m=max(xs) - min(xs) + 2 * inflation,
                ideal_clear_width_m=max(ys) - min(ys) + 2 * inflation,
                command_based_radius_m=travel + inflation,
                final_heading_rad=heading)


def run_ros(args, samples):
    """Publish in wall time; retain the ROS context during the final zero burst."""
    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from rclpy.signals import SignalHandlerOptions

    interrupted = False

    def on_signal(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    old_signals = {sig: signal.signal(sig, on_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = publisher = recorder = None
    started = time.monotonic()
    deadline = started + args.max_duration - STOP_SECONDS
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('response_excitation')
        message_type = TwistStamped if args.message_type == 'TwistStamped' else Twist

        def check():
            if interrupted:
                raise KeyboardInterrupt
            if time.monotonic() > deadline:
                raise RuntimeError('maximum total duration reached')
            if recorder is not None and recorder.poll() is not None:
                raise RuntimeError('bag recorder exited before completion')
            allowed = 1 if publisher is not None else 0
            if node.count_publishers(args.topic) > allowed:
                raise RuntimeError(f'another publisher exists on {args.topic}')
            types = dict(node.get_topic_names_and_types()).get(args.topic, [])
            if any(t != f'geometry_msgs/msg/{args.message_type}' for t in types):
                raise RuntimeError('command topic has a conflicting message type')

        def wait_until(end):
            while time.monotonic() < end:
                check()
                rclpy.spin_once(node, timeout_sec=min(0.02, max(0, end - time.monotonic())))
            check()

        # Check registered endpoints, including silent publishers, before creating ours.
        wait_until(time.monotonic() + 2.0)
        check()
        publisher = node.create_publisher(message_type, args.topic, 10)

        def publish(v, w):
            msg = message_type()
            twist = msg
            if args.message_type == 'TwistStamped':
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = args.frame_id
                twist = msg.twist
            twist.linear.x, twist.angular.z = float(v), float(w)
            publisher.publish(msg)

        if args.record:
            if args.record.exists():
                raise RuntimeError('recording destination already exists')
            args.record.parent.mkdir(parents=True, exist_ok=True)
            subscribers = publisher.get_subscription_count()
            recorder = subprocess.Popen(['ros2', 'bag', 'record', '--storage', args.storage,
                                         '-o', str(args.record), '--topics', *args.record_topics],
                                        stdout=sys.stderr, stderr=sys.stderr, start_new_session=True)
            wait_until(time.monotonic() + 3.0)
            if publisher.get_subscription_count() <= subscribers:
                raise RuntimeError('recorder did not subscribe to the command topic in time')
        countdown_end = time.monotonic() + args.countdown
        while time.monotonic() < countdown_end:
            print(f'Starting in {math.ceil(countdown_end - time.monotonic())} s; Ctrl-C cancels.', file=sys.stderr)
            wait_until(min(countdown_end, time.monotonic() + 1.0))
        check()
        if publisher.get_subscription_count() == 0:
            print('No command subscribers detected.', file=sys.stderr)
        if args.record:
            (args.record / 'excitation.json').write_text(json.dumps(dict(
                mode=args.mode, topic=args.topic, message_type=args.message_type,
                speed=args.speed, amplitude=args.amplitude, period=args.period,
                linear_accel=args.linear_accel, angular_accel=args.angular_accel,
                estimate=motion_estimate(samples, args.footprint_radius, args.clearance)), indent=2) + '\n')
        # Sleep a full period after each send: no catch-up bursts after scheduler stalls.
        # Slew increments were bounded for this minimum actual elapsed interval.
        for _, v, w in samples:
            check()
            publish(v, w)
            wait_until(time.monotonic() + args.period)
        return 0
    except KeyboardInterrupt:
        print('Interrupted; sending zero commands for one second.', file=sys.stderr)
        return 130
    except (RuntimeError, OSError) as exc:
        print(f'ABORT: {exc}', file=sys.stderr)
        return 2
    finally:
        if publisher is not None:
            # SIGINT/SIGTERM handlers only set a flag, so repeated interrupts do not
            # tear down rclpy during this best-effort software stop.
            end = time.monotonic() + STOP_SECONDS
            while time.monotonic() < end:
                try:
                    publish(0.0, 0.0)
                except Exception as exc:
                    print(f'Zero publish failed: {exc}; use the physical stop.', file=sys.stderr)
                    break
                time.sleep(min(args.period, max(0, end - time.monotonic())))
        if recorder is not None and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:
                recorder.wait(timeout=10)
            except subprocess.TimeoutExpired:
                recorder.terminate()
                try:
                    recorder.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    recorder.kill()
                    recorder.wait(timeout=3)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', required=True)
    parser.add_argument('--message-type', required=True, choices=('Twist', 'TwistStamped'))
    parser.add_argument('--mode', required=True, choices=('spin', 'drive', 'straight', 'sine'))
    parser.add_argument('--speed', type=float, required=True, help='forward target [m/s]; zero for spin')
    parser.add_argument('--amplitude', type=float, required=True, help='yaw rate [rad/s]; zero for straight')
    parser.add_argument('--linear-accel', type=float, required=True, help='forward slew limit [m/s^2]')
    parser.add_argument('--angular-accel', type=float, required=True, help='yaw slew limit [rad/s^2]')
    parser.add_argument('--period', type=float, required=True, help='command period [s]')
    parser.add_argument('--footprint-radius', type=float, required=True, help='circumscribed body radius [m]')
    parser.add_argument('--clearance', type=float, required=True, help='extra clearance around the footprint [m]')
    parser.add_argument('--hold', type=float, default=1.0, help='hold/pulse interval [s]')
    parser.add_argument('--rest', type=float, default=1.0)
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--frequency', type=float, default=0.5, help='sine frequency [Hz]')
    parser.add_argument('--cycles', type=int, default=4, help='sine cycles including two-cycle rise and one-cycle fall')
    parser.add_argument('--countdown', type=float, default=3.0)
    parser.add_argument('--max-duration', type=float, default=60.0, help='wall-time bound from ROS startup through zero stop [s]; bag flushing is separate')
    parser.add_argument('--frame-id', default='base_link')
    parser.add_argument('--record', type=Path, help='new rosbag2 directory')
    parser.add_argument('--record-topics', nargs='+')
    parser.add_argument('--storage', choices=('sqlite3', 'mcap'), default='sqlite3')
    parser.add_argument('--dry-run', action='store_true')
    return parser


def main(argv=None):
    parser = make_parser()
    args = parser.parse_args(argv)
    try:
        samples = build_sequence(args)
    except ValueError as exc:
        parser.error(str(exc))
    estimate = motion_estimate(samples, args.footprint_radius, args.clearance)
    print('Ideal command-following space estimate: ' + json.dumps(estimate), file=sys.stderr)
    print('Open-loop motion: reserve additional measured stopping/lag space, clear the area, and keep a physical stop ready. Ctrl-C sends zero for 1 s.', file=sys.stderr)
    if args.dry_run:
        writer = csv.writer(sys.stdout)
        writer.writerow(('t', 'v', 'w'))
        writer.writerows(samples)
        return 0
    try:
        return run_ros(args, samples)
    except ImportError as exc:
        parser.error(f'live excitation requires a sourced ROS 2 environment: {exc}')


if __name__ == '__main__':
    raise SystemExit(main())
