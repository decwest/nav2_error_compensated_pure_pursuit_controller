// Copyright (c) 2026 Fumiya Ohnishi
// SPDX-License-Identifier: Apache-2.0

#include <cmath>

#include "gtest/gtest.h"
#include "nav2_error_compensated_pure_pursuit_controller/ecpp_math.hpp"

namespace ecpp_math = nav2_error_compensated_pure_pursuit_controller::ecpp_math;
using ecpp_math::GateMode;

namespace
{

nav_msgs::msg::Path makePath(const std::vector<std::pair<double, double>> & points)
{
  nav_msgs::msg::Path path;
  for (const auto & [x, y] : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    path.poses.push_back(pose);
  }
  return path;
}

ecpp_math::EcppParams defaultParams(GateMode mode)
{
  ecpp_math::EcppParams params;
  params.omega_n = 2.0;
  params.zeta = 1.0;
  params.v_epsilon = 0.05;
  params.gate_error_on = 0.10;
  params.gate_error_off = 0.50;
  params.gate_endpoint_value = 0.01;
  params.gate_mode = mode;
  return params;
}

}  // namespace

TEST(EcppGate, EndpointValuesMatchDesign)
{
  // sigma(z_on) = 1 - p and sigma(z_off) = p by construction.
  EXPECT_NEAR(ecpp_math::gateAbs(0.10, 0.10, 0.50, 0.01), 0.99, 1e-6);
  EXPECT_NEAR(ecpp_math::gateAbs(0.50, 0.10, 0.50, 0.01), 0.01, 1e-6);
}

TEST(EcppGate, MonotoneDecreasingBetweenThresholds)
{
  const double a = ecpp_math::gateAbs(0.10, 0.10, 0.50, 0.01);
  const double b = ecpp_math::gateAbs(0.30, 0.10, 0.50, 0.01);
  const double c = ecpp_math::gateAbs(0.50, 0.10, 0.50, 0.01);
  EXPECT_GT(a, b);
  EXPECT_GT(b, c);
}

TEST(EcppGate, ModeFromString)
{
  EXPECT_EQ(ecpp_math::gateModeFromString("ey_only"), GateMode::EY_ONLY);
  EXPECT_EQ(ecpp_math::gateModeFromString("product"), GateMode::PRODUCT);
  EXPECT_EQ(ecpp_math::gateModeFromString("sigmoid"), GateMode::PRODUCT);
  EXPECT_EQ(ecpp_math::gateModeFromString("always_on"), GateMode::ALWAYS_ON);
  EXPECT_EQ(ecpp_math::gateModeFromString("off"), GateMode::OFF);
  EXPECT_THROW(ecpp_math::gateModeFromString("bogus"), std::invalid_argument);
}

TEST(EcppTerms, GateOffMatchesPurePursuit)
{
  const auto params = defaultParams(GateMode::OFF);
  const auto terms = ecpp_math::computeEcppTerms(0.7, 0.1, 0.1, 0.5, 0.5, params);
  EXPECT_DOUBLE_EQ(terms.curvature, 0.7);
  EXPECT_DOUBLE_EQ(terms.sigma, 0.0);
}

TEST(EcppTerms, AlwaysOnAppliesCompensation)
{
  const auto params = defaultParams(GateMode::ALWAYS_ON);
  const auto terms = ecpp_math::computeEcppTerms(0.0, 0.1, 0.1, 0.5, 0.5, params);
  EXPECT_DOUBLE_EQ(terms.sigma, 1.0);
  EXPECT_GT(std::fabs(terms.compensation), 1e-6);
}

TEST(EcppTerms, EyOnlyGateIgnoresHeadingError)
{
  // On the path with a large heading error: the ey-only gate stays open
  // while the product gate closes.
  const auto ey_only = ecpp_math::computeEcppTerms(
    0.0, 0.0, M_PI / 2.0, 0.5, 0.5, defaultParams(GateMode::EY_ONLY));
  EXPECT_NEAR(ey_only.sigma, ey_only.sigma_y, 1e-12);
  EXPECT_GT(ey_only.sigma, 0.98);

  const auto product = ecpp_math::computeEcppTerms(
    0.0, 0.0, M_PI / 2.0, 0.5, 0.5, defaultParams(GateMode::PRODUCT));
  EXPECT_LT(product.sigma, 0.02);
}

TEST(EcppTerms, EyOnlyGateClosesFarFromPath)
{
  // e_y = 2 * L_d -> epsilon_y = 4 >> gate_error_off.
  const auto terms = ecpp_math::computeEcppTerms(
    0.0, 1.0, 0.0, 0.5, 0.5, defaultParams(GateMode::EY_ONLY));
  EXPECT_LT(terms.sigma, 0.011);
}

TEST(EcppTerms, GainCancellationRecoversPurePursuit)
{
  // Choose omega_n and zeta so that K_y == 2/L_d^2 and K_psi == 2/L_d:
  //   omega_n = sqrt(2) * v_gain / L_d, zeta = 1/sqrt(2).
  const double l_d = 0.5;
  const double v = 0.5;
  const double v_epsilon = 0.05;
  const double v_gain = v + v_epsilon;
  auto params = defaultParams(GateMode::ALWAYS_ON);
  params.omega_n = std::sqrt(2.0) * v_gain / l_d;
  params.zeta = 1.0 / std::sqrt(2.0);
  params.v_epsilon = v_epsilon;

  const auto terms = ecpp_math::computeEcppTerms(0.42, 0.2, 0.3, l_d, v, params);
  EXPECT_NEAR(terms.dK_y, 0.0, 1e-9);
  EXPECT_NEAR(terms.dK_psi, 0.0, 1e-9);
  EXPECT_NEAR(terms.curvature, 0.42, 1e-9);
}

TEST(EcppTerms, NominalLookaheadSetsPurePursuitGainAndLateralGateError)
{
  auto params = defaultParams(GateMode::EY_ONLY);
  params.omega_n = 1.0;
  params.v_epsilon = 0.05;
  const double nominal_lookahead_dist = 2.0;
  const double e_y = 0.8;

  // v_gain = 0.95 + 0.05 = 1, so K_y = 1. The PP-equivalent gain must use
  // the nominal lookahead: Delta K_y = 1 - 2 / 2^2 = 0.5.
  const auto terms = ecpp_math::computeEcppTerms(
    0.4, e_y, 0.1, nominal_lookahead_dist, 0.95, params);
  const double expected_epsilon_y =
    (e_y / nominal_lookahead_dist) * (e_y / nominal_lookahead_dist);

  EXPECT_NEAR(terms.dK_y, 0.5, 1e-12);
  EXPECT_NEAR(
    terms.sigma_y,
    ecpp_math::gateAbs(
      expected_epsilon_y, params.gate_error_on, params.gate_error_off,
      params.gate_endpoint_value),
    1e-12);
  EXPECT_DOUBLE_EQ(terms.lookahead_dist, nominal_lookahead_dist);
}

TEST(EcppTerms, ErrorCompensationOptionSelectsCorrectCurvature)
{
  const auto terms = ecpp_math::computeEcppTerms(
    0.4, 0.2, 0.1, 0.6, 0.5, defaultParams(GateMode::ALWAYS_ON));
  ASSERT_NE(terms.curvature, terms.kappa_pp);

  EXPECT_DOUBLE_EQ(ecpp_math::selectRegulationCurvature(terms, false), terms.kappa_pp);
  EXPECT_DOUBLE_EQ(ecpp_math::selectRegulationCurvature(terms, true), terms.curvature);
}

TEST(EcppGoldenVector, MatchesConfiguredGainAndSharedProjectionDefinition)
{
  // Same golden vector as the canonical Python implementation. The robot pose
  // (0, 0.15, 0) makes the world-frame path y=0 appear at y=-0.15 in base_link.
  const auto transformed_path = makePath({{-1.0, -0.15}, {2.0, -0.15}});
  const auto projection =
    nav2_error_compensated_pure_pursuit_controller::arc_length_lookahead::projectPath(
    transformed_path);
  ASSERT_TRUE(projection.valid);
  EXPECT_EQ(projection.segment_index, 0u);
  EXPECT_NEAR(projection.interpolation_ratio, 1.0 / 3.0, 1e-12);
  EXPECT_NEAR(projection.arc_length, 1.0, 1e-12);
  EXPECT_NEAR(projection.position.x, 0.0, 1e-12);
  EXPECT_NEAR(projection.position.y, -0.15, 1e-12);
  EXPECT_NEAR(projection.tangent_x, 1.0, 1e-12);
  EXPECT_NEAR(projection.tangent_y, 0.0, 1e-12);
  EXPECT_NEAR(projection.remaining_arc_length, 2.0, 1e-12);

  constexpr double kLookahead = 0.5;
  const auto carrot =
    nav2_error_compensated_pure_pursuit_controller::arc_length_lookahead::getLookAheadPoint(
    kLookahead, transformed_path, projection);
  EXPECT_NEAR(carrot.pose.position.x, 0.5, 1e-12);
  EXPECT_NEAR(carrot.pose.position.y, -0.15, 1e-12);

  const auto path_error = ecpp_math::computePathFrameError(projection);
  ASSERT_TRUE(path_error.valid);
  EXPECT_NEAR(path_error.e_y, 0.15, 1e-12);
  EXPECT_NEAR(path_error.e_psi, 0.0, 1e-12);

  const double kappa_pp =
    2.0 * carrot.pose.position.y /
    (carrot.pose.position.x * carrot.pose.position.x +
    carrot.pose.position.y * carrot.pose.position.y);
  EXPECT_NEAR(kappa_pp, -0.3 / (0.25 + 0.0225), 1e-12);

  ecpp_math::EcppParams params;
  params.omega_n = 1.1892;
  params.zeta = 1.0;
  params.v_epsilon = 0.05;
  params.gate_mode = GateMode::ALWAYS_ON;
  const auto terms = ecpp_math::computeEcppTerms(
    kappa_pp, path_error.e_y, path_error.e_psi, kLookahead, 0.5, params);

  const double expected_v_gain = 0.55;
  const double expected_k_y = std::pow(1.1892 / expected_v_gain, 2.0);
  const double expected_k_psi = 2.0 * 1.1892 / expected_v_gain;
  const double expected_dk_y = expected_k_y - 2.0 / (kLookahead * kLookahead);
  const double expected_dk_psi = expected_k_psi - 2.0 / kLookahead;
  const double expected_curvature = kappa_pp - expected_dk_y * 0.15;

  EXPECT_NEAR(terms.v_gain, expected_v_gain, 1e-12);
  EXPECT_NEAR(terms.dK_y, expected_dk_y, 1e-12);
  EXPECT_NEAR(terms.dK_psi, expected_dk_psi, 1e-12);
  EXPECT_DOUBLE_EQ(terms.sigma, 1.0);
  EXPECT_NEAR(terms.kappa_pp, kappa_pp, 1e-12);
  EXPECT_NEAR(terms.curvature, expected_curvature, 1e-12);
}

TEST(EcppGoldenVector, MatchesNonzeroHeadingAndEyOnlyGate)
{
  // World path [(-1, 0), (2, 0)] expressed in the frame of robot pose
  // (x, y, yaw) = (0, 0.15, 0.2).
  constexpr double kYaw = 0.2;
  constexpr double kLateralError = 0.15;
  const double cosine = std::cos(kYaw);
  const double sine = std::sin(kYaw);
  const auto world_to_robot = [&](const double world_x) {
      return std::pair<double, double>{
      cosine * world_x - sine * kLateralError,
      -sine * world_x - cosine * kLateralError};
    };
  const auto transformed_path = makePath(
    {world_to_robot(-1.0), world_to_robot(2.0)});

  const auto projection =
    nav2_error_compensated_pure_pursuit_controller::arc_length_lookahead::projectPath(
    transformed_path);
  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.arc_length, 1.0, 1e-12);
  EXPECT_NEAR(projection.position.x, -sine * kLateralError, 1e-12);
  EXPECT_NEAR(projection.position.y, -cosine * kLateralError, 1e-12);
  EXPECT_NEAR(projection.tangent_x, cosine, 1e-12);
  EXPECT_NEAR(projection.tangent_y, -sine, 1e-12);

  constexpr double kLookahead = 0.5;
  const auto carrot =
    nav2_error_compensated_pure_pursuit_controller::arc_length_lookahead::getLookAheadPoint(
    kLookahead, transformed_path, projection);
  const double expected_carrot_x = cosine * kLookahead - sine * kLateralError;
  const double expected_carrot_y = -sine * kLookahead - cosine * kLateralError;
  EXPECT_NEAR(carrot.pose.position.x, expected_carrot_x, 1e-12);
  EXPECT_NEAR(carrot.pose.position.y, expected_carrot_y, 1e-12);

  const auto path_error = ecpp_math::computePathFrameError(projection);
  ASSERT_TRUE(path_error.valid);
  EXPECT_NEAR(path_error.e_y, kLateralError, 1e-12);
  EXPECT_NEAR(path_error.e_psi, kYaw, 1e-12);

  const double kappa_pp =
    2.0 * expected_carrot_y /
    (kLookahead * kLookahead + kLateralError * kLateralError);
  ecpp_math::EcppParams params;
  params.omega_n = 1.1892;
  params.zeta = 1.0;
  params.v_epsilon = 0.05;
  params.gate_error_on = 0.10;
  params.gate_error_off = 0.50;
  params.gate_endpoint_value = 0.01;
  params.gate_mode = GateMode::EY_ONLY;
  const auto terms = ecpp_math::computeEcppTerms(
    kappa_pp, path_error.e_y, path_error.e_psi, kLookahead, 0.5, params);

  const double expected_v_gain = 0.55;
  const double expected_dk_y = std::pow(1.1892 / expected_v_gain, 2.0) - 8.0;
  const double expected_dk_psi = 2.0 * 1.1892 / expected_v_gain - 4.0;
  const double expected_sigma = ecpp_math::gateAbs(
    std::pow(kLateralError / kLookahead, 2.0), 0.10, 0.50, 0.01);
  const double expected_curvature = kappa_pp - expected_sigma *
    (expected_dk_y * kLateralError + expected_dk_psi * std::sin(kYaw));

  EXPECT_NEAR(terms.v_gain, expected_v_gain, 1e-12);
  EXPECT_NEAR(terms.sigma, expected_sigma, 1e-12);
  EXPECT_NEAR(terms.dK_y, expected_dk_y, 1e-12);
  EXPECT_NEAR(terms.dK_psi, expected_dk_psi, 1e-12);
  EXPECT_NEAR(terms.kappa_pp, kappa_pp, 1e-12);
  EXPECT_NEAR(terms.curvature, expected_curvature, 1e-12);
}

TEST(PathFrameError, StraightOffsetPath)
{
  // Path along +x at y = -0.2 (robot at origin is 0.2 m LEFT of the path).
  const auto path = makePath({{-0.5, -0.2}, {0.5, -0.2}, {1.5, -0.2}});
  const auto err = ecpp_math::computePathFrameError(path, 2.0);
  ASSERT_TRUE(err.valid);
  EXPECT_NEAR(err.e_y, 0.2, 1e-9);
  EXPECT_NEAR(err.e_psi, 0.0, 1e-9);
}

TEST(PathFrameError, RightOfPathIsNegative)
{
  const auto path = makePath({{-0.5, 0.3}, {1.5, 0.3}});
  const auto err = ecpp_math::computePathFrameError(path, 2.0);
  ASSERT_TRUE(err.valid);
  EXPECT_NEAR(err.e_y, -0.3, 1e-9);
}

TEST(PathFrameError, HeadingErrorSign)
{
  // Path at +45 degrees through the origin: robot heading (0) minus path
  // tangent (+45 deg) -> e_psi = -pi/4.
  const auto path = makePath({{-0.5, -0.5}, {0.5, 0.5}});
  const auto err = ecpp_math::computePathFrameError(path, 2.0);
  ASSERT_TRUE(err.valid);
  EXPECT_NEAR(err.e_psi, -M_PI / 4.0, 1e-9);
  EXPECT_NEAR(err.e_y, 0.0, 1e-9);
}

TEST(PathFrameError, StoredPoseOrientationsDoNotAffectProjectionTangent)
{
  auto path = makePath({{-0.5, -0.2}, {1.5, -0.2}});
  path.poses.front().pose.orientation.z = 1.0;
  path.poses.front().pose.orientation.w = 0.0;
  path.poses.back().pose.orientation.z = std::sin(M_PI / 8.0);
  path.poses.back().pose.orientation.w = std::cos(M_PI / 8.0);

  const auto projection =
    nav2_error_compensated_pure_pursuit_controller::arc_length_lookahead::projectPath(path);
  const auto err = ecpp_math::computePathFrameError(projection);
  ASSERT_TRUE(err.valid);
  EXPECT_NEAR(projection.tangent_x, 1.0, 1e-12);
  EXPECT_NEAR(projection.tangent_y, 0.0, 1e-12);
  EXPECT_NEAR(err.e_y, 0.2, 1e-12);
  EXPECT_NEAR(err.e_psi, 0.0, 1e-12);
}

TEST(PathFrameError, DegenerateInputs)
{
  EXPECT_FALSE(ecpp_math::computePathFrameError(makePath({}), 2.0).valid);
  EXPECT_FALSE(ecpp_math::computePathFrameError(makePath({{0.0, 0.0}}), 2.0).valid);
  // Duplicate points only -> no usable segment.
  EXPECT_FALSE(
    ecpp_math::computePathFrameError(makePath({{0.1, 0.0}, {0.1, 0.0}}), 2.0).valid);
}

TEST(ErrorFilter, TauZeroIsPassThrough)
{
  ecpp_math::FirstOrderFilter filter;
  filter.tau = 0.0;
  EXPECT_DOUBLE_EQ(filter.update(0.3, 0.05), 0.3);
  EXPECT_DOUBLE_EQ(filter.update(-1.2, 0.05), -1.2);
}

TEST(ErrorFilter, StepResponseMatchesFirstOrderLag)
{
  // Discrete first-order lag: after each step y_{k+1} = y_k + dt/(tau+dt)(x - y_k),
  // so y_k = 1 - (tau/(tau+dt))^k for a unit step from zero initial state.
  ecpp_math::FirstOrderFilter filter;
  filter.tau = 0.1;
  const double dt = 0.05;
  filter.update(0.0, dt);  // initialize at 0
  const double a = filter.tau / (filter.tau + dt);
  double expected = 0.0;
  for (int k = 1; k <= 20; ++k) {
    const double y = filter.update(1.0, dt);
    expected = 1.0 - std::pow(a, k);
    EXPECT_NEAR(y, expected, 1e-12);
  }
  EXPECT_GT(expected, 0.99);  // converged near the input
}

TEST(ErrorFilter, ResetReinitializesAtNextSample)
{
  ecpp_math::FirstOrderFilter filter;
  filter.tau = 0.2;
  filter.update(0.0, 0.05);
  filter.update(1.0, 0.05);
  filter.reset();
  // After reset the first sample initializes the state exactly.
  EXPECT_DOUBLE_EQ(filter.update(0.7, 0.05), 0.7);
}

TEST(ErrorFilter, AngleFilterHandlesWrapAround)
{
  ecpp_math::FirstOrderAngleFilter filter;
  filter.tau = 0.1;
  const double dt = 0.05;
  filter.update(3.1, dt);  // initialize just below +pi
  // Input just below -pi: the short way is across +-pi, not through zero.
  const double y = filter.update(-3.1, dt);
  // Innovation is wrap(-3.1 - 3.1) = +0.083... so the state should move
  // toward +pi and beyond (wrapped), never through zero.
  EXPECT_GT(std::fabs(y), 3.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
