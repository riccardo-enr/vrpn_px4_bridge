// Copyright 2026 Riccardo Enrico
// SPDX-License-Identifier: BSD-3-Clause

#ifndef VRPN_PX4_BRIDGE__FRAME_TRANSFORMS_HPP_
#define VRPN_PX4_BRIDGE__FRAME_TRANSFORMS_HPP_

#include <array>
#include <cmath>
#include <limits>

namespace vrpn_px4_bridge
{

/*
 * Frame conversion: mocap world -> PX4 NED.
 *
 * `vrpn_mocap` is a raw passthrough: it publishes the VRPN pose verbatim, with
 * no frame conversion (see its `Tracker::HandlePose`). So the axes you receive
 * are exactly what the mocap software (Vicon Tracker / OptiTrack Motive) is
 * configured to broadcast. Configure the tracker so its world frame is
 * **ENU** (x=East, y=North, z=Up), which is the convention assumed here.
 *
 * PX4 `VehicleOdometry` with `POSE_FRAME_NED` expects **NED**
 * (x=North, y=East, z=Down). The map ENU -> NED is:
 *   position: (N, E, D) = (y, x, -z)          -- swap x/y, negate z
 * That transform swaps two axes and negates one: determinant +1, i.e. a proper
 * rotation, so the same permutation applied to a quaternion's vector part
 * preserves the physical rotation:
 *   quaternion: (w, x, y, z) -> (w, y, x, -z)
 *
 * This is a coordinate RE-EXPRESSION of the same rotation (a similarity
 * transform R_ned = C R_enu C^T with C the ENU->NED matrix). It does NOT add a
 * separate FLU->FRD body-frame flip -- correct when the rigid body was created
 * world-aligned in the mocap software. Verify heading in the arena.
 */

inline std::array<float, 3> enu_to_ned_pos(double x, double y, double z)
{
  return {static_cast<float>(y), static_cast<float>(x), static_cast<float>(-z)};
}

inline std::array<float, 4> enu_to_ned_quat(double w, double x, double y, double z)
{
  // PX4 quaternion order is [w, x, y, z].
  return {
    static_cast<float>(w), static_cast<float>(y),
    static_cast<float>(x), static_cast<float>(-z)};
}

/*
 * PX4 rejects non-unit attitude and treats a NaN first element as "invalid".
 * vrpn can emit slightly non-unit quaternions (float noise) or a degenerate
 * zero/garbage quaternion on tracking loss. Normalize a usable quaternion;
 * otherwise return the explicit PX4 "invalid" marker so EKF2 drops the sample
 * instead of fusing garbage.
 */
inline std::array<float, 4> normalize_or_invalidate(const std::array<float, 4> & q)
{
  const double n2 = static_cast<double>(q[0]) * q[0] + static_cast<double>(q[1]) * q[1] +
    static_cast<double>(q[2]) * q[2] + static_cast<double>(q[3]) * q[3];
  if (!std::isfinite(n2) || n2 < 1e-12) {
    return {std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 0.0f};
  }
  const float inv = static_cast<float>(1.0 / std::sqrt(n2));
  return {q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv};
}

/*
 * PX4 treats a NaN position component as "invalid". A non-finite (inf) input
 * would otherwise corrupt EKF2, so coerce any non-finite component to NaN.
 */
inline std::array<float, 3> sanitize_pos(std::array<float, 3> p)
{
  for (auto & v : p) {
    if (!std::isfinite(v)) {v = std::numeric_limits<float>::quiet_NaN();}
  }
  return p;
}

}  // namespace vrpn_px4_bridge

#endif  // VRPN_PX4_BRIDGE__FRAME_TRANSFORMS_HPP_
