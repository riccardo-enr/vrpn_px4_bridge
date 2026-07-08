# vrpn_px4_bridge

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

```shell
colcon test --packages-select vrpn_px4_bridge
```
