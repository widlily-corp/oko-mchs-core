#!/usr/bin/env python3
"""Сквозная проверка: зрение Ромы → автопилот Артёма → протокол ЦУКС.

Публикует синтетический кадр пожара и одометрию, слушает
``/mchs/detection_event`` и ``/mchs/mission_status``. Успех — когда FSM
перешёл в ``HOVER_INSPECTION`` и автопилот записал ``incident_protocol.json``.

Запускать при уже работающих нодах зрения и автопилота:
    python3 scripts/ros_pilot_integration_check.py --timeout 40
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from mchs_vision.simulated_scene import (NADIR_ALTITUDE_M, SCENE_YAW_DEG,
                                         SCENES, SyntheticArena)

EXPECTED_XY = (42.0, 98.0)


class PilotVisionChecker(Node):
    def __init__(self, frame: np.ndarray, timeout_s: float,
                 report_dir: Optional[str]):
        super().__init__("mchs_pilot_integration_check")
        self.bridge = CvBridge()
        self.frame = frame
        self.timeout_s = timeout_s
        self.report_dir = report_dir

        self.image_publisher = self.create_publisher(
            Image, "/mchs/camera/image_raw", qos_profile_sensor_data
        )
        self.odom_publisher = self.create_publisher(Odometry, "/mchs/odom", 10)
        self.create_subscription(String, "/mchs/detection_event",
                                 self._on_event, 10)
        self.create_subscription(String, "/mchs/mission_status",
                                 self._on_status, 10)

        self.done = False
        self.success = False
        self.reason = "таймаут: автопилот не перешёл в HOVER_INSPECTION"
        self.event_count = 0
        self.status_count = 0
        self.last_phase = ""
        self.protocol_json: Optional[str] = None
        self.last_event = None

    def publish_sample(self) -> None:
        stamp = self.get_clock().now().to_msg()

        image_msg = self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = "camera_optical_link"
        self.image_publisher.publish(image_msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = EXPECTED_XY[0]
        odom.pose.pose.position.y = EXPECTED_XY[1]
        odom.pose.pose.position.z = NADIR_ALTITUDE_M
        yaw = math.radians(SCENE_YAW_DEG)
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.odom_publisher.publish(odom)

    def _on_event(self, msg: String) -> None:
        self.event_count += 1
        try:
            self.last_event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.last_event = None

    def _on_status(self, msg: String) -> None:
        self.status_count += 1
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        phase = str(status.get("phase", ""))
        self.last_phase = phase
        self.protocol_json = status.get("protocol_json")

        if phase != "HOVER_INSPECTION":
            return

        # Статус публикуется раз в секунду; даём протоколу записаться.
        if self.protocol_json and os.path.isfile(self.protocol_json):
            self.success = True
            self.reason = (
                f"HOVER_INSPECTION, событий={self.event_count}, "
                f"протокол={os.path.basename(self.protocol_json)}"
            )
            self.done = True


def build_frame() -> np.ndarray:
    arena = SyntheticArena()
    arena.render()
    return arena.crop(SCENES["fire_critical"].center_px)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--report-dir", default=None)
    args = parser.parse_args()

    frame = build_frame()
    rclpy.init()
    checker = PilotVisionChecker(frame, args.timeout, args.report_dir)
    start = time.monotonic()
    next_publish = 0.0
    seen_hover_at: Optional[float] = None

    try:
        while rclpy.ok() and not checker.done:
            now = time.monotonic()
            elapsed = now - start
            if elapsed > args.timeout:
                break
            if now >= next_publish:
                checker.publish_sample()
                next_publish = now + 0.1
            rclpy.spin_once(checker, timeout_sec=0.02)

            if checker.last_phase == "HOVER_INSPECTION" and seen_hover_at is None:
                seen_hover_at = elapsed
            if seen_hover_at is not None and elapsed - seen_hover_at > 15.0:
                break
    finally:
        elapsed = time.monotonic() - start
        checker.destroy_node()
        rclpy.shutdown()

    if checker.success:
        print(f"[PASS] Сквозная интеграция: {checker.reason}; "
              f"статусов={checker.status_count}, время={elapsed:.1f} с")
        return 0

    print(f"[FAIL] Сквозная интеграция: {checker.reason}; "
          f"фаза={checker.last_phase or 'нет'}, событий={checker.event_count}, "
          f"статусов={checker.status_count}, время={elapsed:.1f} с",
          file=sys.stderr)
    if checker.last_event:
        print("Последнее событие: "
              f"{json.dumps(checker.last_event, ensure_ascii=False)[:400]}",
              file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
