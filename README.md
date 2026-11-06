# Error-Compensated Pure Pursuit for Nav2

ROS 2 Lyrical controller plugin for **Error-Compensated Pure Pursuit (ECPP)**,
built on Nav2's Regulated Pure Pursuit (RPP).

ECPP adds a gated error-feedback compensation to the Pure Pursuit (PP) curvature so that a natural frequency and a damping ratio shape the transient response independently of the lookahead distance, while the robot still approaches the path from large tracking errors as PP does.

## Installation

Assumes ROS 2 Lyrical and Nav2 are already installed.

1. Clone this unpublished release branch from its local checkout into your
   workspace's `src/` directory:

   ```bash
   cd ~/ros2_ws/src
   git clone --branch lyrical-release-20261107 \
     /home/decwest/decwest_workspace/nav2_error_compensated_pure_pursuit_controller
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
max_linear_vel: 0.5
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
| `ecpp.v_gain_source` | `commanded` | Gain speed: pre-DWPP regulated request (`commanded`), odometry (`measured`), or configured speed (`nominal`) |
| `ecpp.publish_debug` | `false` | Publish diagnostics on `<controller_id>/ecpp_debug` |

The relative linearization error is `(e_y/lookahead_dist)^2`, derived for a
straight path with zero heading error. The defaults correspond to 10% and 50%.
Other parameters are inherited from Lyrical RPP. The plugin uses C++20,
`nav2_ros_common`, and the distribution's `dynamic_window_pure_pursuit::computeDynamicWindowVelocities`.
Set `use_dynamic_window: true` to apply upstream DWPP after ECPP compensation;
`false` returns the raw curvature request. `ecpp.gate_mode: off` selects PP
with either window setting. Collision checking uses the final command.
`newPathReceived()` retains the raw plan; projection, carrot, and errors share
the same transformed projection.

- Use a positive `max_linear_vel` and set `use_fixed_curvature_lookahead: false` for ECPP.
- Configure velocity and acceleration limits in the downstream Nav2 velocity smoother.
  For DWPP, also set the inherited linear/angular velocity and signed acceleration limits.
- During reverse motion, the controller uses PP without error compensation.

### Nominal gain speed

`ecpp.v_gain_source: nominal` uses the configured `max_linear_vel` as `v0`,
retaining it during speed limits, regulation, and DWPP deceleration. Dynamic
updates of `max_linear_vel` change `v0`; `setSpeedLimit()` only changes the
requested speed. The math adds `v_epsilon` once and internally converts
`omega_n` to `omega_n * (v0 + v_epsilon) / v0`. Thus nominal gains are exactly
`Ky = (omega_n / v0)^2` and `Ktheta = 2*zeta*omega_n/v0`. Supply the physical
design frequency directly; remove any external epsilon correction when
migrating older experiment profiles.

`commanded` uses the speed before DWPP, so a DWPP slowdown alone does not
distinguish it from `nominal`. Regulation, approach, speed limits, or cancellation
can distinguish them. `measured` uses the absolute odometry speed. Speed and
acceleration limits remain those of upstream DWPP and the downstream smoother.


## Testing

After building, run these commands from the workspace root:

```bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 colcon test --packages-select nav2_error_compensated_pure_pursuit_controller
colcon test-result --verbose
```

## Lyrical deployment

The public paper configurations below retain their experiment values. Lyrical
uses `max_linear_vel` (`desired_linear_vel` is a deprecated upstream alias) and
Controller Server path-handler plugins such as `nav2_controller::FeasiblePathHandler`.
Set `use_dynamic_window` explicitly; the paper configurations set it to `false`
to preserve the original PP/ECPP comparisons. For WHILL's unstamped `Twist`, set
`enable_stamped_cmd_vel: false` on Controller Server and downstream consumers;
otherwise retain Lyrical's stamped default. The local transform tolerance comes
from the costmap. `max_robot_pose_search_dist` remains a startup setting here.

## Parameter migration

`ecpp.use_error_compensation` is replaced by `ecpp.gate_mode` (`off` or an active
gate). `ecpp.error_filter_tau`, `ecpp.error_search_window`, and `ecpp.omega_max`
are removed; the projection search uses `max_robot_pose_search_dist`, and DWPP
limits use `max_angular_vel`/`min_angular_vel`. `gate_mode: product` is removed;
select `ey_only` for the lateral gate. Old undeclared YAML overrides can be
silently unused by ROS, so remove them when updating a deployment.

## Paper experiments

- [Experiment 1](config/paper_experiment1_params.yaml): straight-path tracking
  with PP and six ECPP gain settings.
- [Experiment 2](config/paper_experiment2_params.yaml): indoor path tracking
  with PP, RPP, ECPP, MPPI, and DWB.

Merge these controller settings into your robot's Nav2 configuration.
The experiment configurations disable PP-family collision checking; the
[general example](config/example_param.yaml) enables it. Other controller
settings in experiment 2 originate from the Jazzy paper and may need their own
Lyrical migration (in particular MPPI); they are not ECPP runtime validation.

<!-- ## Reference

F. Ohnishi and M. Takahashi, "Error-Compensated Pure Pursuit for Transient
Response Shaping and Convergence from Large Tracking Errors," 2026.

The paper's simulation code is in [ecpp](https://github.com/decwest/ecpp). -->

## License

[Apache-2.0](LICENSE). The controller and path handler adapt code from
[Nav2 Regulated Pure Pursuit](https://github.com/ros-navigation/navigation2/tree/lyrical/nav2_regulated_pure_pursuit_controller);
upstream copyright notices are retained in the derived files.
