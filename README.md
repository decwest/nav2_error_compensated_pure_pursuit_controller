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

The implementation inherits `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
(Jazzy API), uses a plugin-local path handler to retain the active segment's
predecessor, and replaces the carrot and command-synthesis steps, mirroring
[nav2_dynamic_window_pure_pursuit_controller](https://github.com/Decwest/nav2_dynamic_window_pure_pursuit_controller).
RPP speed regulation, rotate-to-heading, and collision checking are reused;
lookahead selection is the continuous arc-length implementation described above.

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

Requires ROS 2 Jazzy with `ros-jazzy-navigation2` (the RPP base-class API of
the Jazzy release line).

## Example configuration

See [config/example_ecpp_params.yaml](config/example_ecpp_params.yaml).

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
