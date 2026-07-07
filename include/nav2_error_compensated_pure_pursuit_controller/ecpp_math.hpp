// Copyright (c) 2026 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#ifndef NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__ECPP_MATH_HPP_
#define NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__ECPP_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "angles/angles.h"
#include "nav_msgs/msg/path.hpp"

namespace nav2_error_compensated_pure_pursuit_controller
{
namespace ecpp_math
{

inline double logistic(double x)
{
  if (x >= 0.0) {
    const double z = std::exp(-x);
    return 1.0 / (1.0 + z);
  }
  const double z = std::exp(x);
  return z / (1.0 + z);
}

// Decreasing sigmoid gate on a non-negative error rate z_abs:
// ~ (1 - endpoint) at z_on, 0.5 at the midpoint, ~ endpoint at z_off.
inline double gateAbs(double z_abs, double z_on, double z_off, double endpoint_value)
{
  const double center = 0.5 * (z_on + z_off);
  const double slope =
    2.0 * std::log((1.0 - endpoint_value) / endpoint_value) / (z_off - z_on);
  return logistic(-slope * (z_abs - center));
}

enum class GateMode { EY_ONLY, PRODUCT, ALWAYS_ON, OFF };

inline GateMode gateModeFromString(const std::string & mode)
{
  if (mode == "ey_only") {return GateMode::EY_ONLY;}
  if (mode == "product" || mode == "sigmoid") {return GateMode::PRODUCT;}
  if (mode == "always_on") {return GateMode::ALWAYS_ON;}
  if (mode == "off") {return GateMode::OFF;}
  throw std::invalid_argument(
          "ecpp gate_mode must be one of: ey_only, product, always_on, off (got: " + mode + ")");
}

struct PathFrameError
{
  double e_y{0.0};
  double e_psi{0.0};
  bool valid{false};
};

// First-order low-pass filter for the error signals fed to the compensation.
// This is the standard derivative-filtering practice for noise-sensitive
// error-feedback channels; tau <= 0 disables the filter (pass-through).
struct FirstOrderFilter
{
  double tau{0.0};
  bool initialized{false};
  double y{0.0};

  double update(double x, double dt)
  {
    if (tau <= 0.0) {
      initialized = true;
      y = x;
      return y;
    }
    if (!initialized) {
      initialized = true;
      y = x;
      return y;
    }
    y += (dt / (tau + dt)) * (x - y);
    return y;
  }

  void reset()
  {
    initialized = false;
    y = 0.0;
  }
};

// Angle-aware variant: the innovation is wrapped so that filtering behaves
// correctly across the +-pi boundary.
struct FirstOrderAngleFilter
{
  double tau{0.0};
  bool initialized{false};
  double y{0.0};

  double update(double x, double dt)
  {
    if (tau <= 0.0) {
      initialized = true;
      y = angles::normalize_angle(x);
      return y;
    }
    if (!initialized) {
      initialized = true;
      y = angles::normalize_angle(x);
      return y;
    }
    const double innovation = angles::normalize_angle(x - y);
    y = angles::normalize_angle(y + (dt / (tau + dt)) * innovation);
    return y;
  }

  void reset()
  {
    initialized = false;
    y = 0.0;
  }
};

// Computes the signed lateral error e_y and heading error e_psi of the robot
// with respect to the transformed plan expressed in the robot base frame
// (robot at the origin with zero heading). The nearest point is found by
// projecting the origin onto consecutive path segments within the first
// search_window_m of path arc length. Path pose orientations are NOT used;
// the tangent comes from point differences (global planners such as NavFn
// emit identity orientations on intermediate poses).
inline PathFrameError computePathFrameError(
  const nav_msgs::msg::Path & transformed_plan,
  double search_window_m)
{
  PathFrameError result;
  const auto & poses = transformed_plan.poses;
  if (poses.size() < 2) {
    return result;
  }

  double best_dist2 = std::numeric_limits<double>::max();
  double best_proj_x = 0.0, best_proj_y = 0.0;
  double best_tx = 1.0, best_ty = 0.0;
  double arc_length = 0.0;
  bool found = false;

  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    const double ax = poses[i].pose.position.x;
    const double ay = poses[i].pose.position.y;
    const double bx = poses[i + 1].pose.position.x;
    const double by = poses[i + 1].pose.position.y;
    const double sx = bx - ax;
    const double sy = by - ay;
    const double seg_len2 = sx * sx + sy * sy;
    const double seg_len = std::sqrt(seg_len2);
    if (seg_len < 1e-9) {
      continue;
    }
    // Project the origin (robot) onto the segment, clamped to [0, 1].
    double t = -(ax * sx + ay * sy) / seg_len2;
    t = std::clamp(t, 0.0, 1.0);
    const double px = ax + t * sx;
    const double py = ay + t * sy;
    const double dist2 = px * px + py * py;
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      best_proj_x = px;
      best_proj_y = py;
      best_tx = sx / seg_len;
      best_ty = sy / seg_len;
      found = true;
    }
    arc_length += seg_len;
    if (arc_length > search_window_m && found) {
      break;
    }
  }

  if (!found) {
    return result;
  }
  // Left-of-path positive: e_y = n . (robot - proj) with n = (-ty, tx),
  // robot at the origin.
  result.e_y = best_ty * best_proj_x - best_tx * best_proj_y;
  result.e_psi = angles::normalize_angle(-std::atan2(best_ty, best_tx));
  result.valid = true;
  return result;
}

struct EcppParams
{
  double omega_n{1.0};
  double zeta{1.0};
  double v_epsilon{0.05};
  double gate_error_on{0.10};
  double gate_error_off{0.50};
  double gate_endpoint_value{0.01};
  GateMode gate_mode{GateMode::EY_ONLY};
};

struct EcppTerms
{
  double curvature{0.0};
  double kappa_pp{0.0};
  double compensation{0.0};
  double sigma{0.0};
  double sigma_y{0.0};
  double sigma_psi{0.0};
  double e_y{0.0};
  double e_psi{0.0};
  double dK_y{0.0};
  double dK_psi{0.0};
  double v_gain{0.0};
  double lookahead_dist{0.0};
};

// Port of calc_ecpp_terms in the ECPP reference implementation
// (ecpp/src/ecpp/controllers/ecpp.py):
//   kappa = kappa_pp - sigma * (dK_y * e_y + dK_psi * sin(e_psi))
// with dK_y = (omega_n / v)^2 - 2/L_d^2, dK_psi = 2 zeta omega_n / v - 2/L_d
// and a gate sigma driven by relative linearization error rates.
inline EcppTerms computeEcppTerms(
  double kappa_pp, double e_y, double e_psi,
  double lookahead_dist, double v_speed, const EcppParams & params)
{
  constexpr double kEps = 1e-12;
  EcppTerms terms;
  const double l_d = std::max(lookahead_dist, kEps);
  const double v_gain = std::fabs(v_speed) + std::max(params.v_epsilon, kEps);

  const double k_y = (params.omega_n / v_gain) * (params.omega_n / v_gain);
  const double k_psi = 2.0 * params.zeta * params.omega_n / v_gain;
  const double k_y_pp = 2.0 / (l_d * l_d);
  const double k_psi_pp = 2.0 / l_d;
  const double dK_y = k_y - k_y_pp;
  const double dK_psi = k_psi - k_psi_pp;

  const double sin_e_psi = std::sin(e_psi);
  const double epsilon_y = (e_y / l_d) * (e_y / l_d);
  double epsilon_psi = 0.0;
  if (std::fabs(e_psi) > kEps) {
    epsilon_psi = std::fabs(e_psi - sin_e_psi) / std::max(std::fabs(sin_e_psi), kEps);
  }

  double sigma_y = 1.0;
  double sigma_psi = 1.0;
  double sigma = 1.0;
  switch (params.gate_mode) {
    case GateMode::ALWAYS_ON:
      sigma_y = 1.0;
      sigma_psi = 1.0;
      sigma = 1.0;
      break;
    case GateMode::OFF:
      sigma_y = 0.0;
      sigma_psi = 0.0;
      sigma = 0.0;
      break;
    case GateMode::EY_ONLY:
      sigma_y = gateAbs(
        epsilon_y, params.gate_error_on, params.gate_error_off, params.gate_endpoint_value);
      sigma_psi = gateAbs(
        epsilon_psi, params.gate_error_on, params.gate_error_off, params.gate_endpoint_value);
      sigma = sigma_y;
      break;
    case GateMode::PRODUCT:
    default:
      sigma_y = gateAbs(
        epsilon_y, params.gate_error_on, params.gate_error_off, params.gate_endpoint_value);
      sigma_psi = gateAbs(
        epsilon_psi, params.gate_error_on, params.gate_error_off, params.gate_endpoint_value);
      sigma = sigma_y * sigma_psi;
      break;
  }

  const double compensation_raw = dK_y * e_y + dK_psi * sin_e_psi;
  const double compensation = -sigma * compensation_raw;

  terms.curvature = kappa_pp + compensation;
  terms.kappa_pp = kappa_pp;
  terms.compensation = compensation;
  terms.sigma = sigma;
  terms.sigma_y = sigma_y;
  terms.sigma_psi = sigma_psi;
  terms.e_y = e_y;
  terms.e_psi = e_psi;
  terms.dK_y = dK_y;
  terms.dK_psi = dK_psi;
  terms.v_gain = v_gain;
  terms.lookahead_dist = l_d;
  return terms;
}

}  // namespace ecpp_math
}  // namespace nav2_error_compensated_pure_pursuit_controller

#endif  // NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__ECPP_MATH_HPP_
