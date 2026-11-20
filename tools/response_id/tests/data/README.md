# Measured regression data

These two numeric series are unoccupied and occupied identification runs on an
example electric wheelchair at 0.5 m/s, recorded on 2026-11-10. Each file is the
first run of its load condition. They contain only elapsed time, forward/yaw
commands before and after the command pipeline, and measured forward/yaw speed.
No poses, map, device identifiers, absolute timestamps or internal paths are
included. The series retain all 200 Hz samples; columns use the public CSV names
and numbers are written with nine significant digits.

Per-run reference values (body response, then pipeline-inclusive limit at 30 Hz):

| File | K | L [s] | T [s] | Pipeline L [s] | 30-degree limit [rad/s] |
| --- | ---: | ---: | ---: | ---: | ---: |
| unoccupied.csv | 0.9281 | 0.030 | 0.060 | 0.035 | 3.062 |
| occupied.csv | 0.8258 | 0.000 | 0.170 | 0.040 | 2.297 |

The README's repeated-run figures additionally use a second run per condition.
The tests allow gain error 0.02, total body delay error 0.01 s, limit error
0.1 rad/s and phase-margin error 2 degrees. Small timestamp-grid rounding in the
original exports is intentionally retained to detect accidental resampling.

Run the ROS-free tests from the repository root:

```bash
python3 -m pip install -r tools/response_id/requirements.txt pytest rosbags
python3 -m pytest -q tools/response_id/tests
```

The synthetic bag tests use `rosbags`, not ROS. Without that optional dependency,
only those two tests are skipped.
