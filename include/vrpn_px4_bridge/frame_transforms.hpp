// Copyright 2026 Riccardo Enrico
// SPDX-License-Identifier: BSD-3-Clause

#ifndef VRPN_PX4_BRIDGE__FRAME_TRANSFORMS_HPP_
#define VRPN_PX4_BRIDGE__FRAME_TRANSFORMS_HPP_

#include <array>

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

}  // namespace vrpn_px4_bridge

#endif  // VRPN_PX4_BRIDGE__FRAME_TRANSFORMS_HPP_
