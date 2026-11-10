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
