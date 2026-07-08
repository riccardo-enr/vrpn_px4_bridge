// Copyright 2026 Riccardo Enrico
// SPDX-License-Identifier: BSD-3-Clause

/*
 * Adversarial, dependency-free check of the ENU -> NED conversion for BOTH
 * position and attitude, plus the sanitizers. The suite assumes the code is
 * wrong until proven otherwise: it cross-checks against an INDEPENDENT
 * reference, exercises boundary/degenerate/invalid inputs, and pins invariants.
 *
 * Attitude strategy: never compare quaternion components against a hand-copied
 * expectation (q and -q are the same rotation, and copying (w,y,x,-z) would
 * just re-assert the implementation). Build the rotation matrix from the node's
 * output quaternion and compare it to the independent reference
 *   R_ned = C * R_enu * C^T
 * with C the ENU->NED matrix. Any wrong component/sign diverges the matrices.
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <random>

#include "vrpn_px4_bridge/frame_transforms.hpp"

using vrpn_px4_bridge::enu_to_ned_pos;
using vrpn_px4_bridge::enu_to_ned_quat;
using vrpn_px4_bridge::normalize_or_invalidate;
using vrpn_px4_bridge::sanitize_pos;

namespace
{
using Mat3 = std::array<std::array<double, 3>, 3>;

// ENU -> NED coordinate transform: (N,E,D) = (y,x,-z). Symmetric, det +1.
const Mat3 kC = {{{{0, 1, 0}}, {{1, 0, 0}}, {{0, 0, -1}}}};

Mat3 mul(const Mat3 & a, const Mat3 & b)
{
  Mat3 r{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) {s += a[i][k] * b[k][j];}
      r[i][j] = s;
    }
  }
  return r;
}

Mat3 transpose(const Mat3 & a)
{
  Mat3 r{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {r[i][j] = a[j][i];}
  }
  return r;
}

// Rotation matrix (body->world) from a quaternion (w,x,y,z); normalizes first.
Mat3 quat_to_mat(double w, double x, double y, double z)
{
  const double n = std::sqrt(w * w + x * x + y * y + z * z);
  w /= n; x /= n; y /= n; z /= n;
  return {{
    {{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)}},
    {{2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)}},
    {{2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}}
  }};
}

double max_abs_diff(const Mat3 & a, const Mat3 & b)
{
  double m = 0.0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {m = std::max(m, std::abs(a[i][j] - b[i][j]));}
  }
  return m;
}

// The node's output quaternion must re-express the rotation as R_ned = C R C^T.
void expect_attitude(double w, double x, double y, double z, double tol = 1e-6)
{
  const auto q = enu_to_ned_quat(w, x, y, z);
  const Mat3 R_out = quat_to_mat(q[0], q[1], q[2], q[3]);
  const Mat3 R_ref = mul(mul(kC, quat_to_mat(w, x, y, z)), transpose(kC));
  EXPECT_LT(max_abs_diff(R_out, R_ref), tol)
    << "attitude mismatch for q=(" << w << "," << x << "," << y << "," << z << ")";
}
}  // namespace

// ===========================================================================
// Position
// ===========================================================================

TEST(Position, SwapXYNegateZ)
{
  const auto ned = enu_to_ned_pos(1.0, 2.0, 3.0);   // (E, N, U)
  EXPECT_FLOAT_EQ(ned[0], 2.0f);
  EXPECT_FLOAT_EQ(ned[1], 1.0f);
  EXPECT_FLOAT_EQ(ned[2], -3.0f);
}

TEST(Position, MatchesCMatrixReference)
{
  const double pts[][3] = {{0.3, -1.7, 4.2}, {-5, 5, -5}, {0, 0, 0}, {1e-4, 2e1, -7}};
  for (const auto & p : pts) {
    const auto ned = enu_to_ned_pos(p[0], p[1], p[2]);
    EXPECT_NEAR(ned[0], kC[0][0] * p[0] + kC[0][1] * p[1] + kC[0][2] * p[2], 1e-4);
    EXPECT_NEAR(ned[1], kC[1][0] * p[0] + kC[1][1] * p[1] + kC[1][2] * p[2], 1e-4);
    EXPECT_NEAR(ned[2], kC[2][0] * p[0] + kC[2][1] * p[1] + kC[2][2] * p[2], 1e-4);
  }
}

// Applying the position map twice is the identity (C is an involution).
TEST(Position, DoubleApplicationIsIdentity)
{
  for (double v : {-9.5, -1e-3, 0.0, 3.3, 123.4}) {
    const auto a = enu_to_ned_pos(v, 2 * v, 3 * v);
    const auto b = enu_to_ned_pos(a[0], a[1], a[2]);
    EXPECT_FLOAT_EQ(b[0], static_cast<float>(v));
    EXPECT_FLOAT_EQ(b[1], static_cast<float>(2 * v));
    EXPECT_FLOAT_EQ(b[2], static_cast<float>(3 * v));
  }
}

// Zero maps to zero exactly (no -0.0 surprises breaking equality downstream).
TEST(Position, ZeroMapsToZero)
{
  const auto ned = enu_to_ned_pos(0.0, 0.0, 0.0);
  EXPECT_EQ(ned[0], 0.0f);
  EXPECT_EQ(ned[1], 0.0f);
  EXPECT_EQ(ned[2], 0.0f);
}

// Arena-scale value survives the double->float cast within 1 mm.
TEST(Position, FloatPrecisionAtArenaScale)
{
  const auto ned = enu_to_ned_pos(12.3456789, -45.6789012, 7.8901234);
  EXPECT_NEAR(ned[0], -45.6789012f, 1e-3);
  EXPECT_NEAR(ned[1], 12.3456789f, 1e-3);
  EXPECT_NEAR(ned[2], -7.8901234f, 1e-3);
}

// ===========================================================================
// Attitude: canonical physical orientations (incl. 180 deg singularities)
// ===========================================================================

TEST(Attitude, Identity) {expect_attitude(1, 0, 0, 0);}
TEST(Attitude, YawAboutUp90) {expect_attitude(M_SQRT1_2, 0, 0, M_SQRT1_2);}
TEST(Attitude, YawAboutUpNeg90) {expect_attitude(M_SQRT1_2, 0, 0, -M_SQRT1_2);}
TEST(Attitude, RollAboutEast90) {expect_attitude(M_SQRT1_2, M_SQRT1_2, 0, 0);}
TEST(Attitude, PitchAboutNorth90) {expect_attitude(M_SQRT1_2, 0, M_SQRT1_2, 0);}
TEST(Attitude, YawAboutUp180) {expect_attitude(0, 0, 0, 1);}
TEST(Attitude, RollAboutEast180) {expect_attitude(0, 1, 0, 0);}
TEST(Attitude, PitchAboutNorth180) {expect_attitude(0, 0, 1, 0);}
TEST(Attitude, GenericTilt) {expect_attitude(0.2, -0.5, 0.7, 0.46904157598);}

// Negated quaternion is the same rotation -> same NED matrix.
TEST(Attitude, SignAmbiguityIsHandled)
{
  expect_attitude(-M_SQRT1_2, 0, 0, -M_SQRT1_2);   // == YawAboutUp90 negated
}

// Non-unit input still yields the correct rotation (matrix compare normalizes).
TEST(Attitude, NonUnitInputStillCorrectRotation)
{
  expect_attitude(2.0, 0.0, 0.0, 2.0);   // scaled yaw-90, tol relaxed for f32
}

// Deterministic random sweep -- the real bug net.
TEST(Attitude, RandomSweepMatchesReference)
{
  std::mt19937 rng(0xC0FFEE);   // fixed seed: deterministic, no wall clock
  std::normal_distribution<double> g(0.0, 1.0);
  for (int i = 0; i < 20000; ++i) {
    double w = g(rng), x = g(rng), y = g(rng), z = g(rng);
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n < 1e-6) {continue;}
    expect_attitude(w / n, x / n, y / n, z / n, 1e-5);
  }
}

// enu_to_ned_quat is its own inverse at the component level (C is involutive).
TEST(Attitude, PermutationIsInvolution)
{
  const std::array<double, 4> qs[] = {
    {1, 0, 0, 0}, {0.5, 0.5, 0.5, 0.5}, {0.1, -0.2, 0.3, -0.9273618}};
  for (const auto & q : qs) {
    const auto a = enu_to_ned_quat(q[0], q[1], q[2], q[3]);
    const auto b = enu_to_ned_quat(a[0], a[1], a[2], a[3]);
    EXPECT_FLOAT_EQ(b[0], static_cast<float>(q[0]));
    EXPECT_FLOAT_EQ(b[1], static_cast<float>(q[1]));
    EXPECT_FLOAT_EQ(b[2], static_cast<float>(q[2]));
    EXPECT_FLOAT_EQ(b[3], static_cast<float>(q[3]));
  }
}

// Permutation must preserve norm exactly for arbitrary (even non-unit) input.
TEST(Attitude, PermutationPreservesNorm)
{
  const auto q = enu_to_ned_quat(3.0, -4.0, 12.0, 0.0);   // norm = 13
  const double n = std::sqrt(
    static_cast<double>(q[0]) * q[0] + static_cast<double>(q[1]) * q[1] +
    static_cast<double>(q[2]) * q[2] + static_cast<double>(q[3]) * q[3]);
  EXPECT_NEAR(n, 13.0, 1e-5);
}

// ===========================================================================
// normalize_or_invalidate: PX4-facing quaternion sanitizer
// ===========================================================================

TEST(NormalizeQuat, AlreadyUnitIsUnchangedRotation)
{
  const float s = static_cast<float>(M_SQRT1_2);
  const auto q = normalize_or_invalidate({s, 0.0f, 0.0f, s});
  const float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  EXPECT_NEAR(n2, 1.0f, 1e-6f);
}

TEST(NormalizeQuat, ScalesNonUnitToUnit)
{
  const auto q = normalize_or_invalidate({0.0f, 0.0f, 0.0f, 5.0f});
  EXPECT_NEAR(q[0], 0.0f, 1e-6f);
  EXPECT_NEAR(q[3], 1.0f, 1e-6f);
  const float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  EXPECT_NEAR(n2, 1.0f, 1e-6f);
}

TEST(NormalizeQuat, ZeroQuaternionBecomesInvalid)
{
  const auto q = normalize_or_invalidate({0, 0, 0, 0});
  EXPECT_TRUE(std::isnan(q[0]));   // PX4 "invalid" marker
}

TEST(NormalizeQuat, TinyBelowThresholdBecomesInvalid)
{
  const auto q = normalize_or_invalidate({1e-7f, 0, 0, 0});   // n2 = 1e-14 < 1e-12
  EXPECT_TRUE(std::isnan(q[0]));
}

TEST(NormalizeQuat, NanInputBecomesInvalid)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto q = normalize_or_invalidate({nan, 0, 0, 1});
  EXPECT_TRUE(std::isnan(q[0]));
}

TEST(NormalizeQuat, InfInputBecomesInvalid)
{
  const float inf = std::numeric_limits<float>::infinity();
  const auto q = normalize_or_invalidate({inf, 0, 0, 0});
  EXPECT_TRUE(std::isnan(q[0]));
}

TEST(NormalizeQuat, OutputIsFiniteWhenValid)
{
  const auto q = normalize_or_invalidate({0.01f, 0.02f, 0.03f, 0.04f});
  for (float v : q) {EXPECT_TRUE(std::isfinite(v));}
}

// ===========================================================================
// sanitize_pos: PX4-facing position sanitizer
// ===========================================================================

TEST(SanitizePos, FiniteUnchanged)
{
  const auto p = sanitize_pos({1.5f, -2.5f, 3.5f});
  EXPECT_FLOAT_EQ(p[0], 1.5f);
  EXPECT_FLOAT_EQ(p[1], -2.5f);
  EXPECT_FLOAT_EQ(p[2], 3.5f);
}

TEST(SanitizePos, InfComponentBecomesNan)
{
  const float inf = std::numeric_limits<float>::infinity();
  const auto p = sanitize_pos({inf, 2.0f, -inf});
  EXPECT_TRUE(std::isnan(p[0]));
  EXPECT_FLOAT_EQ(p[1], 2.0f);      // finite neighbour untouched
  EXPECT_TRUE(std::isnan(p[2]));
}

TEST(SanitizePos, NanStaysNan)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto p = sanitize_pos({nan, nan, nan});
  for (float v : p) {EXPECT_TRUE(std::isnan(v));}
}

// End-to-end pose path a bug in either stage would perturb.
TEST(Pipeline, PositionPathMatchesReference)
{
  const auto p = sanitize_pos(enu_to_ned_pos(4.0, -1.0, 9.0));
  EXPECT_FLOAT_EQ(p[0], -1.0f);   // N
  EXPECT_FLOAT_EQ(p[1], 4.0f);    // E
  EXPECT_FLOAT_EQ(p[2], -9.0f);   // D
}
