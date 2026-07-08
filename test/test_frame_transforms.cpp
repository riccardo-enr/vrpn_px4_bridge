// Copyright 2026 Riccardo Enrico
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include "vrpn_px4_bridge/frame_transforms.hpp"

using vrpn_px4_bridge::enu_to_ned_pos;
using vrpn_px4_bridge::enu_to_ned_quat;

TEST(FrameTransforms, PositionSwapsXYNegatesZ)
{
  const auto ned = enu_to_ned_pos(1.0, 2.0, 3.0);   // (E, N, U)
  EXPECT_FLOAT_EQ(ned[0], 2.0f);   // N = ENU.y
  EXPECT_FLOAT_EQ(ned[1], 1.0f);   // E = ENU.x
  EXPECT_FLOAT_EQ(ned[2], -3.0f);  // D = -ENU.z
}

TEST(FrameTransforms, QuaternionPermutesVectorPart)
{
  const auto q = enu_to_ned_quat(0.5, 0.1, 0.2, 0.3);  // (w, x, y, z)
  EXPECT_FLOAT_EQ(q[0], 0.5f);   // w unchanged
  EXPECT_FLOAT_EQ(q[1], 0.2f);   // y
  EXPECT_FLOAT_EQ(q[2], 0.1f);   // x
  EXPECT_FLOAT_EQ(q[3], -0.3f);  // -z
}

TEST(FrameTransforms, IdentityQuaternionStaysUnitNorm)
{
  const auto q = enu_to_ned_quat(1.0, 0.0, 0.0, 0.0);
  const float norm2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  EXPECT_FLOAT_EQ(norm2, 1.0f);
}
