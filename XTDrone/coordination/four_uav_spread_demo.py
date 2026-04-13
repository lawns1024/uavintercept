#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time
from typing import Dict, Tuple

import rospy
from geometry_msgs.msg import Pose, Quaternion
from mavros_msgs.msg import State
from std_msgs.msg import String


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


class FourUAVSpreadDemo:
    def __init__(self, uav_num: int, rate_hz: float, cruise_alt: float, auto_land: bool):
        self.uav_num = uav_num
        self.rate = rospy.Rate(rate_hz)
        self.cruise_alt = cruise_alt
        self.auto_land = auto_land

        self.namespaces = [f"iris_{i}" for i in range(uav_num)]
        self.state: Dict[str, State] = {ns: State() for ns in self.namespaces}
        self.cmd_pub: Dict[str, rospy.Publisher] = {}
        self.pose_pub: Dict[str, rospy.Publisher] = {}
        self.targets: Dict[str, Tuple[float, float, float, float]] = {}

        # 4机生成器默认初始点（与你当前 multi_vehicle.launch 一致）
        self.spawn_xy = {
            "iris_0": (0.0, 3.0),
            "iris_1": (3.0, 3.0),
            "iris_2": (0.0, 6.0),
            "iris_3": (3.0, 6.0),
        }

        for ns in self.namespaces:
            self.cmd_pub[ns] = rospy.Publisher(f"/xtdrone/{ns}/cmd", String, queue_size=8)
            self.pose_pub[ns] = rospy.Publisher(f"/xtdrone/{ns}/cmd_pose_enu", Pose, queue_size=12)
            rospy.Subscriber(f"/{ns}/mavros/state", State, self._state_cb(ns), queue_size=1)

            x0, y0 = self.spawn_xy[ns]
            self.targets[ns] = (x0, y0, 0.2, 0.0)

    def _state_cb(self, ns: str):
        def cb(msg: State):
            self.state[ns] = msg

        return cb

    def wait_connections(self, timeout: float = 25.0) -> None:
        t0 = time.time()
        while not rospy.is_shutdown():
            all_connected = all(self.state[ns].connected for ns in self.namespaces)
            if all_connected:
                rospy.loginfo("[demo] All UAV MAVROS connected.")
                return
            if time.time() - t0 > timeout:
                raise RuntimeError("Timeout waiting MAVROS connection for all UAVs")
            self.publish_targets_once()
            self.rate.sleep()

    def publish_targets_once(self):
        for ns in self.namespaces:
            x, y, z, yaw = self.targets[ns]
            msg = Pose()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = z
            msg.orientation = yaw_to_quat(yaw)
            self.pose_pub[ns].publish(msg)

    def stream_setpoints(self, sec: float):
        steps = max(1, int(sec * 30))
        for _ in range(steps):
            self.publish_targets_once()
            self.rate.sleep()

    def _broadcast_cmd(self, cmd: str, sec: float = 1.0):
        rospy.loginfo("[demo] cmd -> %s (%.1fs)", cmd, sec)
        msg = String(data=cmd)
        steps = max(1, int(sec * 10))
        for _ in range(steps):
            for ns in self.namespaces:
                self.cmd_pub[ns].publish(msg)
            self.publish_targets_once()
            self.rate.sleep()

    def _interp_phase(
        self,
        start: Dict[str, Tuple[float, float, float, float]],
        end: Dict[str, Tuple[float, float, float, float]],
        duration: float,
        title: str,
    ):
        rospy.loginfo("[demo] phase: %s (%.1fs)", title, duration)
        steps = max(2, int(duration * 30))
        for k in range(steps):
            a = float(k) / float(steps - 1)
            for ns in self.namespaces:
                s = start[ns]
                e = end[ns]
                self.targets[ns] = (
                    s[0] * (1 - a) + e[0] * a,
                    s[1] * (1 - a) + e[1] * a,
                    s[2] * (1 - a) + e[2] * a,
                    s[3] * (1 - a) + e[3] * a,
                )
            self.publish_targets_once()
            self.rate.sleep()

    def _hold(self, sec: float, title: str):
        rospy.loginfo("[demo] hold: %s (%.1fs)", title, sec)
        self.stream_setpoints(sec)

    def run(self):
        self.wait_connections()

        # official XTDrone chain: cmd topic -> communication bridge -> mavros
        self.stream_setpoints(2.0)
        self._broadcast_cmd("ARM", sec=2.0)
        self._broadcast_cmd("OFFBOARD", sec=2.5)

        # Phase-1: takeoff
        start = dict(self.targets)
        end = {}
        for ns in self.namespaces:
            x0, y0 = self.spawn_xy[ns]
            end[ns] = (x0, y0, self.cruise_alt, 0.0)
        self._interp_phase(start, end, duration=7.0, title="四机起飞")

        # Phase-2: forward flight
        start = dict(self.targets)
        end = {}
        forward_dx = 18.0
        for ns in self.namespaces:
            x, y, z, yaw = self.targets[ns]
            end[ns] = (x + forward_dx, y, z, yaw)
        self._interp_phase(start, end, duration=10.0, title="编队前飞")

        # Phase-3: spread-out capture posture
        # keep center forward, then expand rectangle
        cx = sum(self.targets[ns][0] for ns in self.namespaces) / self.uav_num
        cy = sum(self.targets[ns][1] for ns in self.namespaces) / self.uav_num
        spread_half_w = 8.0
        spread_half_h = 6.0
        spread_targets = {
            "iris_0": (cx - spread_half_w, cy - spread_half_h, self.cruise_alt, 0.0),
            "iris_1": (cx + spread_half_w, cy - spread_half_h, self.cruise_alt, 0.0),
            "iris_2": (cx - spread_half_w, cy + spread_half_h, self.cruise_alt, 0.0),
            "iris_3": (cx + spread_half_w, cy + spread_half_h, self.cruise_alt, 0.0),
        }
        self._interp_phase(dict(self.targets), spread_targets, duration=12.0, title="散开围捕示意")

        # Phase-4: hold for recording
        self._hold(18.0, title="围捕阵型保持（录屏窗口）")

        if self.auto_land:
            self._broadcast_cmd("AUTO.LAND", sec=2.0)
            self._hold(8.0, title="降落中")

        rospy.loginfo("[demo] completed.")


def parse_args():
    parser = argparse.ArgumentParser(description="XTDrone 4-UAV outdoor spread demo (no downwash)")
    parser.add_argument("--uav_num", type=int, default=4)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--alt", type=float, default=4.5)
    parser.add_argument("--auto_land", action="store_true")
    return parser.parse_known_args()[0]


def main():
    args = parse_args()
    rospy.init_node("four_uav_spread_demo", anonymous=True)
    demo = FourUAVSpreadDemo(
        uav_num=args.uav_num,
        rate_hz=args.rate,
        cruise_alt=args.alt,
        auto_land=args.auto_land,
    )
    demo.run()


if __name__ == "__main__":
    main()
