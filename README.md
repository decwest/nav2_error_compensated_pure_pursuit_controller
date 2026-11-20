# Error-Compensated Pure Pursuit for Nav2

ROS 2 Jazzy controller plugin for **Error-Compensated Pure Pursuit (ECPP)**,
built on Nav2's Regulated Pure Pursuit (RPP).

ECPP adds a gated error-feedback compensation to the Pure Pursuit (PP) curvature so that a natural frequency and a damping ratio shape the transient response independently of the lookahead distance, while the robot still approaches the path from large tracking errors as PP does.

## Installation

Assumes ROS 2 Jazzy and Nav2 are already installed.

1. Clone this repository into your workspace's `src/` directory:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/decwest/nav2_error_compensated_pure_pursuit_controller.git
   ```

2. Build the package from the workspace root:

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install --packages-select nav2_error_compensated_pure_pursuit_controller
   source install/setup.bash
   ```

3. Configure Nav2 to use ECPP:

   ```yaml
   controller_server:
     ros__parameters:
       controller_plugins: ["FollowPath"]
       FollowPath:
         plugin: "nav2_error_compensated_pure_pursuit_controller::ErrorCompensatedPurePursuitController"
   ```

   See [config/example_param.yaml](config/example_param.yaml) for an example
   controller configuration.

## Parameters

Set parameters under the controller ID (for example, `FollowPath`).
The [example configuration](config/example_param.yaml) uses:

```yaml
desired_linear_vel: 0.5
lookahead_dist: 1.0
ecpp.omega_n: 1.030
ecpp.zeta: 1.0
```

| Parameter | Default | Meaning |
| --- | --- | --- |
| `ecpp.omega_n` | `1.0` | Natural frequency: controls nominal local response speed [rad/s] |
| `ecpp.zeta` | `1.0` | Damping ratio: controls nominal local damping |
| `ecpp.v_epsilon` | `0.05` | Speed regularization for gain calculation [m/s] |
| `ecpp.gate_mode` | `ey_only` | `ey_only`: lateral-error gate; `always_on`: full compensation; `off`: PP |
| `ecpp.gate_error_on` | `0.10` | Relative linearization error at the ON threshold (gate value `1-p`) |
| `ecpp.gate_error_off` | `0.50` | Relative linearization error at the OFF threshold (gate value `p`) |
| `ecpp.gate_endpoint_value` | `0.01` | Gate endpoint value `p` (`0 < p < 0.5`) |
| `ecpp.v_gain_source` | `commanded` | Speed used for gains: commanded speed or measured odometry (`measured`) |
| `ecpp.publish_debug` | `false` | Publish diagnostics on `<controller_id>/ecpp_debug` |

The relative linearization error is `(e_y/lookahead_dist)^2`, derived for a
straight path with zero heading error. The defaults correspond to 10% and 50%.
Other parameters are inherited from RPP.

- Use a positive `desired_linear_vel` and set `use_fixed_curvature_lookahead: false` for ECPP.
- Configure velocity and acceleration limits in the downstream Nav2 velocity smoother.
- During reverse motion, the controller uses PP without error compensation.


## Choosing `ecpp.omega_n`

Use two design guidelines together:

1. **Angular-velocity constraint.** The paper's reference design value is
   $$\omega_{n,\mathrm{ref}} = \sqrt{\frac{v\,\bar\omega}{\sqrt{\varepsilon_{\mathrm{off}}}\,L_d}},
   \qquad \bar\omega = \omega_{\max} - v|\kappa_{\mathrm{prev}}| > 0.$$
   Here `v` is forward speed, `L_d` is lookahead distance,
   `epsilon_off` is `ecpp.gate_error_off`, and `kappa_prev` is preview curvature.
   This estimates a reference value using the lateral error at the gate's OFF
   threshold. It neglects the heading-error contribution and is not a guarantee
   that the complete command will avoid saturation.
2. **Measured response.** Identify the command-to-motion response and choose a
   frequency with at least **30 degrees of phase margin**, allowing additional
   margin for uncertainty and changing loads. This is a practical local tuning
   guideline, separate from the paper's results. It does not establish global
   stability, collision avoidance, or safe physical stopping.

The standalone tools in [`tools/response_id/`](tools/response_id/) are run from
this checkout and are not installed or linked into the plugin. CSV fitting
requires NumPy: `python3 -m pip install -r tools/response_id/requirements.txt`.
Bag reading additionally needs `python3 -m pip install rosbags`; plotting needs
`matplotlib`. Both analysis tools run without ROS. Live excitation requires a
sourced ROS 2 environment, `rclpy`, the standard geometry messages, and `ros2 bag`
when recording.

**These commands move the robot open loop.** First use `--dry-run` on the
excitation command to export its complete `t,v,w` command sequence to stdout.
The space estimate on stderr integrates the ideal unicycle motion and inflates
its bounds by the supplied circumscribed footprint radius and clearance.
Reserve additional space for measured lag, stopping distance and tracking error;
the estimate cannot establish a safe area for an unmeasured robot. Clear people
and obstacles from the whole swept area, start from rest, and keep an operator
at the physical emergency stop. Ctrl-C or SIGTERM requests zero commands for
one second; normal completion ramps to zero and also sends the zero burst.
An emergency zero command can exceed the normal acceleration ramp. Loss of the
process, communications or power requires an independent watchdog/physical stop.

Use a dedicated input upstream of the velocity smoother and collision monitor.
Remove or remap all other publishers on that input, including idle publishers;
canceling a navigation goal alone is insufficient. The tool checks publisher
endpoints during discovery, countdown and motion, and aborts on a conflict.
This ROS graph check is not an atomic command-ownership lock. Keep the downstream
safety chain active and include its final command and odometry in the recording.
The wall-time limit includes startup and the final zero burst; recorder flushing
has a separate bounded timeout. All unused velocity components, including
lateral velocity, are zero.

The following **example values and topic names are illustrative**; replace them
with the verified limits, footprint and routing for the robot. Run these three
commands from the repository root, using new recording/output directories:

```bash
python3 tools/response_id/excite.py --topic /response_id/cmd_vel --message-type Twist \
  --mode drive --speed 0.2 --amplitude 0.2 --linear-accel 0.3 --angular-accel 0.5 \
  --period 0.05 --footprint-radius 0.35 --clearance 0.3 --max-duration 45 \
  --record response_run --record-topics /response_id/cmd_vel /base/cmd_vel /odom
python3 tools/response_id/fit.py response_run --command-topic /response_id/cmd_vel \
  --body-command-topic /base/cmd_vel --odom-topic /odom --output response_fit
python3 tools/response_id/omega_n_limit.py --model response_fit/model.json \
  --control-period 0.05 --zeta 1 --phase-margin 30 --omega-n 1.5 --speed 0.2
```

`--mode` also accepts `spin` (zero forward speed), `straight` (forward speed
steps, zero yaw amplitude), and `sine` (small cosine with a gradual envelope).
Use `--hold`, `--rest`, `--repeats`, or sine `--frequency`/`--cycles` to adjust
excitation. The tool validates the sine's envelope as well as its sinusoidal
angular acceleration. Use `--message-type TwistStamped` when the input requires
stamped commands; `--frame-id` sets their frame. Motion values, acceleration
limits and command period have no robot-specific defaults.

For CSV input, use columns `t,v_command,w_command,v_odom,w_odom` and optionally
`v_body_command,w_body_command` (seconds, m/s, rad/s). The older names `v_nav`,
`w_nav`, `v_fin`, `w_fin` are accepted as aliases. Column names select the signals
for CSV input; the topic options select signals only for bags. Supply multiple
input files to aggregate repeated trials. Keep load, speed, floor and other
conditions consistent within one fit. The fitter uses bag receipt timestamps,
a 5 ms sampling grid, held commands, and linearly interpolated odometry.
Already uniform CSV series are preserved, including small timestamp rounding.

The first-order-plus-dead-time model is `G_a(s) = K exp(-L s)/(T s + 1)`.
Each trial uses backward-Euler simulation and a least-squares positive gain,
with a 10 ms grid for `L` and `T`; repeated trials use median `K`, `T` and `L+T`.
Change `--dt`, `--time-step`, `--max-delay` and `--max-time-constant` as needed;
an optimum at the upper search bounds is reported. Include rest before motion:
the initial model state assumes a steady initial input, with no fitted offset.
The JSON and terminal table contain `K`, `L`, `T` and R² for both forward and
yaw response; a constant/unexcited axis is reported as unavailable.
`series_*.csv` includes the measured signals and fitted predictions; `--plot`
adds `fit.png`. Inspect these before relying on the result.

With a body-command topic, the recommended model combines its fitted body
response with the median command-to-body delay from cross-correlation, assuming
a unit-gain, pure-delay pipeline. Inspect the separate overall fit if smoothing,
clipping or a safety stop changes the waveform. Without that topic, the tool
fits command to odometry in one stage. Fit JSON excludes sampling delay.
The limit calculator adds half the specified **controller** period exactly once
and evaluates the same local loop for either input method:

$$\mathcal{L}(s) = G_a(s)\,\frac{\omega_n(2\zeta s + \omega_n)}{s^2}.$$

It reports the limit, the candidate's crossover and phase margin, and the
corresponding Pure Pursuit lookahead `sqrt(2) * v / omega_n`. This lookahead is
an equivalent response scale, not an instruction to change ECPP's preview
lookahead. Explicit `--gain`, `--delay` and `--time-constant` may replace JSON;
`--delay` must exclude half the controller period. An ideal delay-free model can
have an unbounded limit; an unattainable target returns a zero limit.

For example, repeated identification runs on **one electric wheelchair** at
0.5 m/s gave the following reported values (30 Hz controller, `zeta = 1`,
pipeline delay approximately 0.0375 s):

| Load | Body K | Body L [s] | Body T [s] | Limit for 30° [rad/s] | Margin at 2.0 rad/s |
| --- | ---: | ---: | ---: | ---: | ---: |
| Unoccupied, two runs | 0.930 | 0.030 | 0.065 | 2.91 (individual runs 2.78–3.06) | 43° |
| Occupied, two runs | 0.828 | 0.000 | 0.175 | 2.33 | 35° |

These are reference measurements, not defaults for another platform. The loaded
response is slower. The compatible backward-Euler fit gives approximately
`K = 0.827, T = 0.170 s` for the occupied recordings and reproduces the reported
limit within 0.1 rad/s; fitted `L` and `T` can trade off. Identify both expected
load conditions and choose using the slower response and the lowest observed
limit. Recheck the response at the intended operating speed.

## Testing

After building, run these commands from the workspace root:

```bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 colcon test --packages-select nav2_error_compensated_pure_pursuit_controller
colcon test-result --verbose
```

<!-- ## Paper experiments

- [Experiment 1](config/paper_experiment1_params.yaml): straight-path tracking
  with PP and six ECPP gain settings.
- [Experiment 2](config/paper_experiment2_params.yaml): indoor path tracking
  with PP, RPP, ECPP, MPPI, and DWB. -->

<!-- Merge these controller settings into your robot's Nav2 configuration.
Experiment conditions and velocity-smoother settings are included as comments.
The experiment configurations disable PP-family collision checking; the
[general example](config/example_param.yaml) enables it. -->

<!-- ## Reference

F. Ohnishi and M. Takahashi, "Error-Compensated Pure Pursuit for Transient
Response Shaping and Convergence from Large Tracking Errors," 2026.

The paper's simulation code is in [ecpp](https://github.com/decwest/ecpp). -->

## License

[Apache-2.0](LICENSE). The controller and path handler adapt code from
[Nav2 Regulated Pure Pursuit](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_regulated_pure_pursuit_controller);
upstream copyright notices are retained in the derived files.

### Jazzy nominal gains

`ecpp.v_gain_source: nominal` fixes gain speed to `base_desired_linear_vel + v_epsilon`.
Speed limits and measured odometry do not retune the gains. Port of Lyrical
11b3b42; the Jazzy public parameter remains `desired_linear_vel`.

### Jazzy EC-DWPP and supervised trials

`use_dynamic_window: true` applies the existing Jazzy DWPP helper to ECPP's
compensated curvature. `dwpp.max_linear_vel`, `dwpp.max_angular_vel`,
`dwpp.max_linear_accel`, and `dwpp.max_angular_accel` are positive configuration-only
limits (defaults 0.5, 1.0, 0.5, 1.5). Motion is forward-only with zero lateral
command. First control after lifecycle reset uses odometry; plan replacement
preserves the previous command. Collision prediction remains enabled separately.
The window limits command increments per configured controller period; physical
acceleration and downstream safety stops require odometry validation.

With `supervised_paths: true`, prepare the pair `ecpp.pending_path_stamp_ns`
(int64 ROS path header timestamp) and `ecpp.pending_omega_n` in one atomic
parameter request. `setPlan` accepts only that newer timestamp, replaces the
projection/search path, and adopts the gain in the same critical section.
`/testbed/path_adopted` is Float64MultiArray containing adoption ROS seconds,
path stamp seconds, path stamp nanoseconds, omega_n, first x, first y.
Action acceptance is not a substitute for this notification.

### Optional closed-loop collision prediction (Jazzy EC-DWPP)

`collision_prediction_mode` defaults to `constant_velocity`, preserving the RPP
constant-command collision check. Select `closed_loop` together with
`use_dynamic_window: true` to predict differential-drive tracking with this
plugin's own arc-length carrot, ECPP gains, speed regulation and DWPP window.
Configuration is read-only after configure; restart the controller to switch.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `collision_prediction_mode` | `constant_velocity` | `constant_velocity` or `closed_loop` |
| `collision_prediction_time_buffer` | `2.0` s | Extra time beyond the maximum stopping time |
| `collision_stop_margin` | `0.3` m | Footprint-sweep distance reserved before first contact |
| `collision_prediction_max_steps` | `300` | Maximum control steps per rollout; insufficient budget rejects control |
| `publish_collision_prediction` | `false` | Publish nominal/verified paths and prediction diagnostics |

The horizon is `max(dt, max(v_max/a_max, w_max/alpha_max,
stop_time(previous_command), stop_time(measured_velocity)) + time_buffer)`.
With 0.5 m/s, 0.5 m/s², 1.0 rad/s and 1.5 rad/s², it is 3 s (90 steps at
30 Hz), independent of carrot distance. The full footprint is checked at
substeps of at most half a costmap cell of translation plus rotational sweep.
The local costmap is held fixed during prediction; leaving its verified extent
is treated as a stopping boundary.

On predicted contact, a second rollout reserves `collision_stop_margin` and
applies common-ratio braking within the reachable DWPP window. Its stopping
envelope includes one control period of command holding. The altered trajectory
is checked again; insufficient braking distance, exhausted step budget or failed
prediction throws `nav2_core::NoValidControl`. This follows the EC-DWVP predictor,
with lateral velocity fixed to zero and ECPP's continuous segment projection.
The prediction uses copies of the plan and controller progress/command state.

When visualization is enabled, `<plugin>/nominal_collision_arc` and
`<plugin>/collision_arc` contain the predicted and verified paths in the costmap
frame. `<plugin>/collision_prediction` reports events, horizon, step counts,
braking distance, speed scale and prediction compute time in milliseconds.
Compute time in this diagnostic excludes context transformation and publishing;
measure the complete `computeVelocityCommands` call for control-period budgeting.
A static snapshot and ideal command following do not model actuator lag or the
future motion of obstacles; downstream collision monitoring remains necessary.
