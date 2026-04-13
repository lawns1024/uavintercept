#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from typing import Dict, Tuple

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


def lerp(a: float, b: float, t: float) -> float:
    return a * (1.0 - t) + b * t


class FourUAVVisualDemo:
    def __init__(self):
        rospy.init_node("four_uav_visual_demo_gazebo", anonymous=True)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=20.0)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.rate = rospy.Rate(30)

        self.models = ["iris_0", "iris_1", "iris_2", "iris_3"]
        self.current: Dict[str, Tuple[float, float, float, float]] = {
            "iris_0": (0.0, 3.0, 0.15, 0.0),
            "iris_1": (3.0, 3.0, 0.15, 0.0),
            "iris_2": (0.0, 6.0, 0.15, 0.0),
            "iris_3": (3.0, 6.0, 0.15, 0.0),
        }

    def _yaw_to_quat(self, yaw: float):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (0.0, 0.0, sy, cy)

    def _publish_pose(self, name: str, x: float, y: float, z: float, yaw: float):
        qx, qy, qz, qw = self._yaw_to_quat(yaw)
        msg = ModelState()
        msg.model_name = name
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = max(0.12, z)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.set_state(msg)

    def _step_to(self, targets: Dict[str, Tuple[float, float, float, float]], duration: float, title: str):
        rospy.loginfo("[visual-demo] phase: %s (%.1fs)", title, duration)
        steps = max(2, int(duration * 30))
        start = dict(self.current)
        for i in range(steps):
            a = float(i) / float(steps - 1)
            for name in self.models:
                s = start[name]
                e = targets[name]
                x = lerp(s[0], e[0], a)
                y = lerp(s[1], e[1], a)
                z = lerp(s[2], e[2], a)
                yaw = lerp(s[3], e[3], a)
                self._publish_pose(name, x, y, z, yaw)
                self.current[name] = (x, y, z, yaw)
            self.rate.sleep()

    def _hold_with_wave(self, sec: float, amp: float = 0.25, title: str = "保持展示"):
        rospy.loginfo("[visual-demo] phase: %s (%.1fs)", title, sec)
        steps = max(1, int(sec * 30))
        base = dict(self.current)
        for k in range(steps):
            t = k / 30.0
            for idx, name in enumerate(self.models):
                x, y, z, yaw = base[name]
                zw = z + amp * math.sin(0.9 * t + 0.8 * idx)
                yw = y + 0.2 * math.cos(0.7 * t + 0.6 * idx)
                self._publish_pose(name, x, yw, zw, yaw)
                self.current[name] = (x, yw, zw, yaw)
            self.rate.sleep()

    def run(self):
        time.sleep(3.0)

        # 1) 起飞
        takeoff = {
            "iris_0": (0.0, 3.0, 4.2, 0.0),
            "iris_1": (3.0, 3.0, 4.2, 0.0),
            "iris_2": (0.0, 6.0, 4.2, 0.0),
            "iris_3": (3.0, 6.0, 4.2, 0.0),
        }
        self._step_to(takeoff, duration=6.0, title="四机起飞")

        # 2) 保持当前水平编队整体前飞
        forward_flat = {
            "iris_0": (10.0, 3.0, 4.4, 0.0),
            "iris_1": (13.0, 3.0, 4.4, 0.0),
            "iris_2": (10.0, 6.0, 4.4, 0.0),
            "iris_3": (13.0, 6.0, 4.4, 0.0),
        }
        self._step_to(forward_flat, duration=7.0, title="水平编队前飞")

        # 3) 扩散飞开，并抬升沿前飞方向的后排两机形成更大倾角网面
        net_tilt_spread = {
            "iris_0": (16.0, -1.5, 8.0, 0.0),
            "iris_1": (24.0, -1.5, 4.2, 0.0),
            "iris_2": (16.0, 10.5, 8.0, 0.0),
            "iris_3": (24.0, 10.5, 4.2, 0.0),
        }
        self._step_to(net_tilt_spread, duration=8.5, title="扩散并形成倾斜网面")

        # 4) 保持倾斜网面形态再向前飞一段
        net_forward = {
            "iris_0": (30.0, -1.5, 8.0, 0.0),
            "iris_1": (38.0, -1.5, 4.2, 0.0),
            "iris_2": (30.0, 10.5, 8.0, 0.0),
            "iris_3": (38.0, 10.5, 4.2, 0.0),
        }
        self._step_to(net_forward, duration=8.0, title="倾斜网面继续前飞")

        # 5) 录屏保持窗口
        self._hold_with_wave(sec=16.0, amp=0.12, title="倾斜网面展示保持")
        rospy.loginfo("[visual-demo] completed.")


if __name__ == "__main__":
    FourUAVVisualDemo().run()
