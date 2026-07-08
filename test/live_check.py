#!/usr/bin/env python3
# Copyright 2026 Riccardo Enrico
# SPDX-License-Identifier: BSD-3-Clause

"""
End-to-end check of a RUNNING vrpn_px4_bridge node, without a live mocap.

It injects synthetic `geometry_msgs/PoseStamped` on the node's input topic and
verifies the emitted `px4_msgs/VehicleOdometry`:
  - position and attitude converted ENU -> NED (attitude checked by comparing
    rotation MATRICES, so the q/-q sign ambiguity cannot mask a bug);
  - pose_frame == POSE_FRAME_NED;
  - velocity / angular_velocity are NaN (EKF2 ignores them);
  - a degenerate (zero) input quaternion produces the PX4 invalid marker
    (q[0] == NaN), NOT garbage;
  - a non-finite input position is coerced to NaN, not passed through as inf;
  - timestamp is non-zero.

Usage (two terminals):
  1) ros2 launch vrpn_px4_bridge vrpn_px4_bridge.launch.py tracker:=drone1
  2) python3 live_check.py --ros-args -p tracker:=drone1

Exit code 0 = all checks passed, 1 = at least one failed.
"""

import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry

# ENU -> NED coordinate transform matrix.
C = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def quat_to_mat(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class LiveCheck(Node):
    def __init__(self):
        super().__init__('vrpn_px4_bridge_live_check')
        tracker = self.declare_parameter('tracker', 'drone1').value
        in_topic = self.declare_parameter(
            'input_topic', f'/vrpn_mocap/{tracker}/pose').value
        out_topic = self.declare_parameter(
            'output_topic', '/fmu/in/vehicle_visual_odometry').value

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(PoseStamped, in_topic, 10)
        self.last = None
        self.create_subscription(VehicleOdometry, out_topic, self._on_odom, px4_qos)
        self.get_logger().info(f'inject {in_topic} -> read {out_topic}')

    def _on_odom(self, msg):
        self.last = msg

    def _pose(self, pos, quat):
        m = PoseStamped()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.pose.orientation.w = float(quat[0])
        m.pose.orientation.x = float(quat[1])
        m.pose.orientation.y = float(quat[2])
        m.pose.orientation.z = float(quat[3])
        return m

    def send_and_get(self, pos, quat, timeout=3.0):
        """Publish one pose repeatedly until a fresh odom arrives."""
        self.last = None
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            self.pub.publish(self._pose(pos, quat))
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last is not None:
                return self.last
        return None


def approx(a, b, tol=1e-4):
    return abs(a - b) <= tol


def run(node):
    checks = []

    def check(name, ok, detail=''):
        checks.append(ok)
        print(f'[{"PASS" if ok else "FAIL"}] {name}' + (f'  {detail}' if detail else ''))

    # Wait for the node to be discoverable.
    if node.count_subscribers(node.pub.topic_name) == 0:
        print('waiting for the bridge node to subscribe...')
        t = time.time() + 5.0
        while time.time() < t and node.count_subscribers(node.pub.topic_name) == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
    if node.count_subscribers(node.pub.topic_name) == 0:
        print('ERROR: no subscriber on the input topic. Is the bridge running '
              'with a matching tracker/input_topic?')
        return False

    # --- position: ENU (E,N,U)=(1,2,3) -> NED (N,E,D)=(2,1,-3) ---
    o = node.send_and_get((1.0, 2.0, 3.0), (1.0, 0.0, 0.0, 0.0))
    if o is None:
        print('ERROR: no VehicleOdometry received. Check output_topic / QoS.')
        return False
    check('position ENU->NED',
          approx(o.position[0], 2.0) and approx(o.position[1], 1.0)
          and approx(o.position[2], -3.0),
          f'got {list(o.position)}')
    check('pose_frame == NED', o.pose_frame == VehicleOdometry.POSE_FRAME_NED,
          f'got {o.pose_frame}')
    check('velocity is NaN', all(math.isnan(v) for v in o.velocity))
    check('angular_velocity is NaN', all(math.isnan(v) for v in o.angular_velocity))
    check('timestamp non-zero', o.timestamp != 0, f'got {o.timestamp}')

    # --- attitude sweep: compare rotation matrices (sign-safe) ---
    cases = {
        'identity': (1, 0, 0, 0),
        'yaw+90 (Up)': (math.sqrt(0.5), 0, 0, math.sqrt(0.5)),
        'roll+90 (East)': (math.sqrt(0.5), math.sqrt(0.5), 0, 0),
        'pitch+90 (North)': (math.sqrt(0.5), 0, math.sqrt(0.5), 0),
        'yaw+180': (0, 0, 0, 1),
        'generic': (0.2, -0.5, 0.7, 0.46904157598),
    }
    for name, q in cases.items():
        o = node.send_and_get((0.0, 0.0, 0.0), q)
        r_out = quat_to_mat(o.q[0], o.q[1], o.q[2], o.q[3])
        r_ref = C @ quat_to_mat(*q) @ C.T
        err = float(np.max(np.abs(r_out - r_ref)))
        check(f'attitude {name}', err < 1e-4, f'max|dR|={err:.2e}')

    # --- degenerate quaternion -> PX4 invalid marker, not garbage ---
    o = node.send_and_get((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
    check('zero quaternion -> q[0]=NaN (invalid)', math.isnan(o.q[0]),
          f'got {list(o.q)}')

    # --- non-finite position -> NaN, not inf passthrough ---
    o = node.send_and_get((float('inf'), 1.0, 2.0), (1.0, 0.0, 0.0, 0.0))
    check('inf position -> NaN', math.isnan(o.position[0]),
          f'got {list(o.position)}')

    passed = sum(1 for c in checks if c)
    print(f'\n{passed}/{len(checks)} checks passed')
    return all(checks)


def main():
    rclpy.init()
    node = LiveCheck()
    try:
        ok = run(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
