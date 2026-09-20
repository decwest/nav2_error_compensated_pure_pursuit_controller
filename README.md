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
