// Copyright (c) 2026 Fumiya Ohnishi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__ECPP_MATH_HPP_
#define NAV2_ERROR_COMPENSATED_PURE_PURSUIT_CONTROLLER__ECPP_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "angles/angles.h"
#include "nav_msgs/msg/path.hpp"

#include "nav2_error_compensated_pure_pursuit_controller/arc_length_lookahead.hpp"

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

enum class GateMode { EY_ONLY, ALWAYS_ON, OFF };

inline GateMode gateModeFromString(const std::string & mode)
{
  if (mode == "ey_only") {return GateMode::EY_ONLY;}
  if (mode == "always_on") {return GateMode::ALWAYS_ON;}
  if (mode == "off") {return GateMode::OFF;}
  throw std::invalid_argument(
          "ecpp gate_mode must be one of: ey_only, always_on, off (got: " + mode + ")");
}

struct PathFrameError
{
  double e_y{0.0};
  double e_psi{0.0};
  bool valid{false};
};

// Computes the signed lateral error e_y and heading error e_psi from the same continuous path
// projection used by carrot and cusp selection. The path is in the robot base frame, so the robot
// is at the origin with zero heading. Path pose orientations are not used.
inline PathFrameError computePathFrameError(
  const arc_length_lookahead::PathProjection & projection)
{
  PathFrameError result;
  if (!projection.valid) {
    return result;
  }

  // Left-of-path positive: e_y = n . (robot - proj) with n = (-ty, tx),
  // robot at the origin.
  result.e_y =
    projection.tangent_y * projection.position.x -
    projection.tangent_x * projection.position.y;
  result.e_psi = angles::normalize_angle(
    -std::atan2(projection.tangent_y, projection.tangent_x));
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
  double sigma_psi{1.0};  // No heading-error attenuation.
  double e_y{0.0};
  double e_psi{0.0};
  double dK_y{0.0};
  double dK_psi{0.0};
  double v_gain{0.0};
  double lookahead_dist{0.0};
};

// Gated difference-gain feedback. omega_n is the design value at desired_speed;
// its internal scaling compensates for the additive gain-speed regularization.
// desired_speed must be finite and positive (validated by the controller).
inline EcppTerms computeEcppTerms(
  double kappa_pp, double e_y, double e_psi,
  double lookahead_dist, double v_speed, double desired_speed, const EcppParams & params)
{
  constexpr double kEps = 1e-12;
  EcppTerms terms;
  const double l_d = std::max(lookahead_dist, kEps);
  const double v_gain = std::fabs(v_speed) + params.v_epsilon;

  const double configured_omega_n =
    params.omega_n * (desired_speed + params.v_epsilon) / desired_speed;
  const double k_y = (configured_omega_n / v_gain) * (configured_omega_n / v_gain);
  const double k_psi = 2.0 * params.zeta * configured_omega_n / v_gain;
  const double k_y_pp = 2.0 / (l_d * l_d);
  const double k_psi_pp = 2.0 / l_d;
  const double dK_y = k_y - k_y_pp;
  const double dK_psi = k_psi - k_psi_pp;

  const double sin_e_psi = std::sin(e_psi);
  const double epsilon_y = (e_y / l_d) * (e_y / l_d);
  double sigma = 0.0;
  switch (params.gate_mode) {
    case GateMode::ALWAYS_ON:
      sigma = 1.0;
      break;
    case GateMode::OFF:
      break;
    case GateMode::EY_ONLY:
      sigma = gateAbs(
        epsilon_y, params.gate_error_on, params.gate_error_off, params.gate_endpoint_value);
      break;
  }

  const double compensation_raw = dK_y * e_y + dK_psi * sin_e_psi;
  const double compensation = sigma == 0.0 ? 0.0 : -sigma * compensation_raw;

  terms.curvature = kappa_pp + compensation;
  terms.kappa_pp = kappa_pp;
  terms.compensation = compensation;
  terms.sigma = sigma;
  terms.sigma_y = sigma;
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
