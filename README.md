# vrpn_px4_bridge

[![ci](https://github.com/riccardo-enr/vrpn_px4_bridge/actions/workflows/ci.yml/badge.svg)](https://github.com/riccardo-enr/vrpn_px4_bridge/actions/workflows/ci.yml)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros)](https://docs.ros.org/en/jazzy/)
[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)

ROS 2 C++ node that relays motion-capture pose from
[`vrpn_mocap`](https://github.com/alvinsunyixiao/vrpn_mocap) (Vicon Tracker or
OptiTrack Motive over VRPN) to PX4 as external vision, converting the mocap
**ENU** world frame to PX4 **NED**.

`vrpn_mocap` is a raw passthrough and does no frame conversion, so this node
does it: it subscribes to a `geometry_msgs/PoseStamped` and publishes a
`px4_msgs/VehicleOdometry` (`POSE_FRAME_NED`) on
`/fmu/in/vehicle_visual_odometry`. C++ keeps the callback path fast and
low-latency.

```
Motive/Vicon (VRPN) -> vrpn_mocap -> /vrpn_mocap/<tracker>/pose
                     -> vrpn_px4_bridge -> /fmu/in/vehicle_visual_odometry
                     -> uXRCE-DDS -> PX4 EKF2
```

## Dependencies

- ROS 2 (Humble / Jazzy), `rclcpp`, `geometry_msgs`
- [`px4_msgs`](https://github.com/PX4/px4_msgs) in your workspace
- [`vrpn_mocap`](https://github.com/alvinsunyixiao/vrpn_mocap) running (`ros-$ROS_DISTRO-vrpn-mocap`)

## Build

Clone into an existing ROS 2 workspace's `src/` (alongside `px4_msgs`):

```shell
cd ~/ros2_ws/src
git clone <this-repo> vrpn_px4_bridge
cd ~/ros2_ws
colcon build --packages-select vrpn_px4_bridge
source install/setup.bash
```

## Run

```shell
# 1. mocap driver
ros2 launch vrpn_mocap client.launch.yaml server:=<motive_ip> port:=3883

# 2. this bridge (tracker = rigid-body name in Motive/Vicon)
ros2 launch vrpn_px4_bridge vrpn_px4_bridge.launch.py tracker:=drone1
```

### Parameters

| Parameter      | Default                              | Description                          |
|----------------|--------------------------------------|--------------------------------------|
| `tracker`      | `drone1`                             | vrpn rigid-body name                 |
| `input_topic`  | `/vrpn_mocap/<tracker>/pose`         | override the input pose topic        |
| `output_topic` | `/fmu/in/vehicle_visual_odometry`    | override the PX4 odometry topic      |

## Frames

Set the mocap world frame to **ENU** (x=East, y=North, z=Up). The conversion is
`(N,E,D) = (y, x, -z)` for position and the same axis permutation on the
quaternion vector part (`q=(w,x,y,z) -> (w,y,x,-z)`), see
[`frame_transforms.hpp`](include/vrpn_px4_bridge/frame_transforms.hpp).

> If your Vicon/Motive world is not ENU (e.g. Vicon Tracker defaults to z-up
> x-forward), either reconfigure the object axes in the mocap software to output
> ENU, or adjust `frame_transforms.hpp`. **Verify in the arena:** push the craft
> North and PX4's estimated x should increase; check heading — a wrong yaw
> convention is the classic mocap bug.

Velocity is published as NaN so EKF2 ignores it (pose-only). Extend `on_pose` to
also consume `/vrpn_mocap/<tracker>/twist` if you want velocity fusion.

## PX4 side

- Run the uXRCE-DDS agent: `MicroXRCEAgent udp4 -p 8888`
- Enable external vision in EKF2 (`EKF2_EV_CTRL`); disable GPS aiding for pure
  mocap flight. See [PX4 ROS 2](https://docs.px4.io/main/en/ros/ros2_comm.html).

## Test

**Unit (offline, no mocap, no ROS running).** `test/test_frame_transforms.cpp`
checks position and attitude conversion plus the sanitizers. Attitude is
verified by comparing rotation *matrices* against an independent reference
`R_ned = C R_enu C^T` (sign-safe), with a 20k-case random sweep and degenerate/
NaN/inf boundary cases.

```shell
colcon test --packages-select vrpn_px4_bridge
colcon test-result --verbose
```

**End-to-end (running node, still no mocap).** `test/live_check.py` injects
synthetic poses on the input topic and validates the emitted VehicleOdometry
(conversion, `pose_frame`, NaN velocity, invalid-quaternion handling,
timestamp). Two terminals:

```shell
# terminal 1
ros2 launch vrpn_px4_bridge vrpn_px4_bridge.launch.py tracker:=drone1
# terminal 2
python3 src/vrpn_px4_bridge/test/live_check.py --ros-args -p tracker:=drone1
```

## Editor errors (clangd "file not found")

Red squiggles like `'rclcpp/rclcpp.hpp' file not found` are the editor's
language server not knowing the ROS include paths — not real build errors.
Generate a compile database and point clangd at it:

```shell
colcon build --packages-select vrpn_px4_bridge \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ln -sf ../../build/vrpn_px4_bridge/compile_commands.json \
  src/vrpn_px4_bridge/compile_commands.json
```

Then reload the editor. (`source /opt/ros/$ROS_DISTRO/setup.bash` before
building so the headers exist.)
