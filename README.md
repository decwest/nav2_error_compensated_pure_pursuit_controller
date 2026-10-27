# nav2_error_compensated_pure_pursuit_controller

Nav2 controller plugin for **Error-Compensated Pure Pursuit (ECPP)**.

ECPP keeps the pure pursuit curvature as a base command and adds gated
difference-gain error feedback shaped by the configured `omega_n` and damping
ratio `zeta`:

```
kappa = kappa_pp - sigma(e_y) * [ (K_y - 2/L_d^2) * e_y + (K_psi - 2/L_d) * sin(e_psi) ]
v_g = |v| + 0.05
K_y = (omega_n / v_g)^2,  K_psi = 2 * zeta * omega_n / v_g
```

- `kappa_pp`: pure pursuit curvature to the carrot selected `L_d` metres of
  polyline arc length ahead of the continuous robot projection.
- `L_d`: the configured nominal arc-length lookahead, not the robot-to-carrot
  chord. It is also used in the intrinsic PP difference gains and gate.
- `e_y`, `e_psi`: lateral and heading tracking errors in the path frame,
  computed by projecting the robot onto the transformed plan (path pose
  orientations are not used; tangents come from point differences).
- `sigma(e_y)`: decreasing sigmoid gate driven by the lateral relative
  linearization error `(e_y / L_d)^2`. Near the path the compensation is fully
  active; far from the path the command smoothly reverts to plain pure
  pursuit, preserving its geometric capture behavior. The heading term needs
  no gate: it uses the bounded `sin(e_psi)` form, which matches the exact
  heading dependence of the pure pursuit curvature.

The implementation inherits the **ROS 2 Lyrical**
`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`.
It uses the distribution's RPP parameter handler and
[`dynamic_window_pure_pursuit::computeDynamicWindowVelocities`](https://github.com/ros-navigation/navigation2/blob/1.5.2/nav2_regulated_pure_pursuit_controller/include/nav2_regulated_pure_pursuit_controller/dynamic_window_pure_pursuit_functions.hpp),
without a Navigation2 fork or a local copy of DWPP.

The control order is PP curvature, RPP speed regulation, ECPP curvature
compensation, optional DWPP velocity selection, then collision checking on the
final command. Rotate-to-heading and speed-limit handling use the RPP base class.
The `commanded` gain speed is the absolute regulated speed **before** DWPP;
`measured` uses the absolute odometry speed. DWPP can change the realized
curvature when the acceleration window prevents the requested command, so
constrained tracking responses must be evaluated separately from ECPP alone.

`newPathReceived()` supplies the raw plan to the plugin-local transformer, which
retains the active segment's predecessor for continuous projection and arc-length
lookahead. The Controller Server's processed plan and global goal are used by
the new goal-checker API. This is compatible with the standard FeasiblePathHandler;
it does not implement lane selection or consume lane-modified plans.

## Parameters

In addition to all regulated pure pursuit parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `ecpp.use_error_compensation` | `false` | Enable ECPP compensation; when false the controller uses the continuous-projection PP curvature |
| `ecpp.omega_n` | 1.0 | Configured gain-shaping parameter [rad/s] |
| `ecpp.zeta` | 1.0 | Target damping ratio [-] |
| `ecpp.v_epsilon` | 0.05 | Low-speed regularization added to the gain speed [m/s] |
| `ecpp.gate_mode` | `ey_only` | `ey_only` \| `product` \| `always_on` \| `off` |
| `ecpp.gate_error_on` | 0.10 | Relative linearization error where the gate is ~fully on |
| `ecpp.gate_error_off` | 0.50 | Relative linearization error where the gate is ~fully off |
| `ecpp.gate_endpoint_value` | 0.01 | Gate residual `p` at the on/off thresholds |
| `ecpp.v_gain_source` | `commanded` | Speed used in the gains: this cycle's regulated command (`commanded`) or odometry (`measured`) |
| `ecpp.error_search_window` | 2.0 | Path arc length searched for the nearest segment [m] |
| `ecpp.error_filter_tau` | 0.0 | First-order low-pass time constant [s] applied to the error signals (`e_y`, `e_psi`) fed to the compensation; 0 disables. Standard derivative-filtering practice for noisy localization — try ~0.1 s (cutoff ~1.6 Hz) and keep `omega_n * tau << 1` so the added phase lag stays negligible. The pure pursuit base curvature is never filtered. |

All `ecpp.*` parameters are dynamically reconfigurable
(`ros2 param set /controller_server <plugin>.ecpp.omega_n 2.12`).

Debug topic: `<plugin_name>/ecpp_debug` (`std_msgs/Float64MultiArray`):
`[e_y, e_psi, sigma, sigma_y, sigma_psi, kappa_pp, kappa, v_gain, L_d]`.

## Build

```bash
cd ~/ros2_ws/src
git clone <this repo>
cd ~/ros2_ws
colcon build --symlink-install --packages-select nav2_error_compensated_pure_pursuit_controller
colcon test --packages-select nav2_error_compensated_pure_pursuit_controller
```

Requires ROS 2 Lyrical with `ros-lyrical-navigation2` (Nav2 1.5.2 or a compatible
Lyrical release) and C++20. The Jazzy implementation remains on the existing branch.

## Example configuration

See [config/example_ecpp_params.yaml](config/example_ecpp_params.yaml).
Keep this plugin and the same arc-length lookahead for all four comparisons:

| `ecpp.use_error_compensation` | `use_dynamic_window` | Behavior |
| --- | --- | --- |
| false | false | Arc-length PP with RPP speed regulation |
| true | false | ECPP |
| false | true | DWPP using arc-length PP curvature |
| true | true | DWPP using ECPP curvature |

Use the inherited `max_linear_vel`, `min_linear_vel`, `max_angular_vel`,
`min_angular_vel`, and signed acceleration/deceleration parameters for DWPP.
Lyrical retains `desired_linear_vel` as a deprecated alias; prefer `max_linear_vel`.
For a legacy-only `ecpp.omega_max` startup override, this plugin forwards the
value and subsequent updates to the canonical `max_angular_vel` parameter.
An explicit canonical startup setting takes precedence.

A WHILL deployment using `Twist` must set `enable_stamped_cmd_vel: false` in
Controller Server and downstream command consumers. This is independent of the
controller plugin's `TwistStamped` return type.

## Notes

- The compensation is a forward-drive design; while reversing
  (`allow_reversing` with a cusp behind), the gate is forced off and the
  command falls back to plain pure pursuit.
- Carrot selection, path-frame errors, and cusp search consume one shared
  continuous `PathProjection` per controller cycle. Path pose orientations do
  not define the tangent; non-zero segment differences do.
- `use_fixed_curvature_lookahead=true` is rejected when ECPP is enabled because
  the compensation and PP base must use the same nominal lookahead.
- Without dynamic-window synthesis, the plugin returns the raw `v * kappa`
  request without an internal angular-velocity clip. Apply platform limits in
  the downstream Nav2 velocity smoother. `ecpp.omega_max` is accepted only as
  a deprecated legacy alias for the DWPP `max_angular_vel` startup setting.
- When less than `L_d` of path remains, the carrot is clamped to the endpoint;
  the local gain guarantee does not cover that endpoint region.

## License

Apache-2.0
