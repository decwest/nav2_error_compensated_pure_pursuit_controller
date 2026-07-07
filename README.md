# nav2_error_compensated_pure_pursuit_controller

Nav2 controller plugin for **Error-Compensated Pure Pursuit (ECPP)**.

ECPP keeps the pure pursuit curvature as a base command and adds gated
difference-gain error feedback so that the local tracking dynamics realize a
desired natural frequency `omega_n` and damping ratio `zeta`:

```
kappa = kappa_pp - sigma(e_y) * [ (K_y - 2/L_d^2) * e_y + (K_psi - 2/L_d) * sin(e_psi) ]
K_y = (omega_n / v)^2,  K_psi = 2 * zeta * omega_n / v
```

- `kappa_pp`: pure pursuit curvature to the lookahead point (chord length `L_d`).
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
(Jazzy API) and replaces only the command-synthesis step, mirroring
[nav2_dynamic_window_pure_pursuit_controller](https://github.com/Decwest/nav2_dynamic_window_pure_pursuit_controller).
All RPP features (lookahead selection, speed regulation, rotate-to-heading,
collision checking) are reused.

## Parameters

In addition to all regulated pure pursuit parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `ecpp.omega_n` | 1.0 | Target natural frequency of the local error dynamics [rad/s] |
| `ecpp.zeta` | 1.0 | Target damping ratio [-] |
| `ecpp.v_epsilon` | 0.05 | Low-speed regularization added to the gain speed [m/s] |
| `ecpp.gate_mode` | `ey_only` | `ey_only` \| `product` \| `always_on` \| `off` |
| `ecpp.gate_error_on` | 0.10 | Relative linearization error where the gate is ~fully on |
| `ecpp.gate_error_off` | 0.50 | Relative linearization error where the gate is ~fully off |
| `ecpp.gate_endpoint_value` | 0.01 | Gate residual `p` at the on/off thresholds |
| `ecpp.omega_max` | 2.0 | Clip for the angular velocity command [rad/s] |
| `ecpp.v_gain_source` | `commanded` | Speed used in the gains: this cycle's regulated command (`commanded`) or odometry (`measured`) |
| `ecpp.error_search_window` | 2.0 | Path arc length searched for the nearest segment [m] |
| `ecpp.error_filter_tau` | 0.0 | First-order low-pass time constant [s] applied to the error signals (`e_y`, `e_psi`) fed to the compensation; 0 disables. Standard derivative-filtering practice for noisy localization — try ~0.1 s (cutoff ~1.6 Hz) and keep `omega_n * tau << 1` so the added phase lag stays negligible. The pure pursuit base curvature is never filtered. |

All `ecpp.*` parameters are dynamically reconfigurable
(`ros2 param set /controller_server <plugin>.ecpp.omega_n 2.12`).

Debug topic: `<plugin_name>/ecpp_debug` (`std_msgs/Float64MultiArray`):
`[e_y, e_psi, sigma, sigma_y, sigma_psi, kappa_pp, kappa, v_gain, L_d_eff]`.

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
- With `use_velocity_scaled_lookahead_dist`, the intrinsic pure pursuit gains
  `2/L_d^2`, `2/L_d` follow the actual chord to the carrot each cycle, so the
  difference gains remain consistent. For controller characterization use a
  fixed lookahead.

## License

Apache-2.0
