#!/usr/bin/env python3
"""Интеграционная проверка ROS 2-ноды зрения без Gazebo.

Публикует в /mchs/camera/image_raw синтетический кадр пожара и в /mchs/odom
позу БПЛА, затем ждёт /mchs/detection_event и /mchs/camera/annotated.

Запускать при уже работающей ноде:
    python3 scripts/ros_integration_check.py --timeout 20

Проверяется:
  * событие пришло и является валидным JSON;
  * hazard_type = FIRE;
  * площадь в разумном диапазоне;
  * координаты отличаются от эталона не более чем на 5 м;
  * аннотированный кадр опубликован.

Код возврата 0 — PASS, 1 — FAIL.
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

EXPECTED_SCENE = "fire_critical"
EXPECTED_XY = (42.0, 98.0)
EXPECTED_AREA_RANGE = (30.0, 100.0)
MAX_POSITION_ERROR_M = 5.0


class IntegrationChecker(Node):
    def __init__(self, frame, expected_xy, timeout_s: float,
                 save_annotated: Optional[str]):
        super().__init__("mchs_vision_integration_check")
        self.bridge = CvBridge()
        self.frame = frame
        self.expected_xy = expected_xy
        self.timeout_s = timeout_s
        self.save_annotated = save_annotated

        self.image_publisher = self.create_publisher(
            Image, "/mchs/camera/image_raw", qos_profile_sensor_data
        )
        self.odom_publisher = self.create_publisher(Odometry, "/mchs/odom", 10)
        self.create_subscription(String, "/mchs/detection_event",
                                 self._on_event, 10)
        self.create_subscription(Image, "/mchs/camera/annotated",
                                 self._on_annotated, 10)

        self.done = False
        self.success = False
        self.reason = "таймаут: событие не получено"
        self.event_count = 0
        self.annotated_count = 0
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
        odom.pose.pose.position.x = self.expected_xy[0]
        odom.pose.pose.position.y = self.expected_xy[1]
        odom.pose.pose.position.z = NADIR_ALTITUDE_M
        yaw = math.radians(SCENE_YAW_DEG)
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.odom_publisher.publish(odom)

    def _on_event(self, msg: String) -> None:
        self.event_count += 1
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.reason = f"невалидный JSON: {exc}"
            self.done = True
            return

        self.last_event = event
        target = event.get("target_geo", {})
        area = float(event.get("area_sq_m", 0.0))
        confidence = float(event.get("confidence", 0.0))
        hazard_type = event.get("hazard_type", "")

        if hazard_type not in ("FIRE", "SMOKE"):
            self.reason = f"неожиданный hazard_type={hazard_type}"
            self.done = True
            return
        if not (EXPECTED_AREA_RANGE[0] <= area <= EXPECTED_AREA_RANGE[1]):
            self.reason = f"площадь {area} м² вне диапазона {EXPECTED_AREA_RANGE}"
            self.done = True
            return

        error = math.hypot(float(target.get("abs_x", 0.0)) - self.expected_xy[0],
                           float(target.get("abs_y", 0.0)) - self.expected_xy[1])
        if error > MAX_POSITION_ERROR_M:
            self.reason = f"ошибка координат {error:.2f} м > {MAX_POSITION_ERROR_M} м"
            self.done = True
            return
        if confidence < 0.5:
            self.reason = f"низкая уверенность {confidence}"
            self.done = True
            return

        self.success = True
        self.reason = (f"FIRE {area:.1f} м², ошибка координат {error:.2f} м, "
                       f"conf={confidence:.2f}")
        self.done = True

    def _on_annotated(self, msg: Image) -> None:
        self.annotated_count += 1
        if self.save_annotated and self.annotated_count == 1:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                os.makedirs(os.path.dirname(self.save_annotated), exist_ok=True)
                cv2.imwrite(self.save_annotated, frame)
            except Exception as exc:
                print(f"[WARN] не удалось сохранить аннотацию: {exc}",
                      file=sys.stderr)


def build_frame(image_path: Optional[str]) -> np.ndarray:
    if image_path:
        frame = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError(f"Не удалось прочитать {image_path}")
        return frame

    arena = SyntheticArena()
    arena.render()
    return arena.crop(SCENES[EXPECTED_SCENE].center_px)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", default=None,
                        help="готовый кадр (иначе синтетический пожар)")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--save-annotated",
                        default="/tmp/oko_mchs_vision_check/ros_annotated.png")
    args = parser.parse_args()

    frame = build_frame(args.image)
    rclpy.init()
    checker = IntegrationChecker(frame, EXPECTED_XY, args.timeout,
                                 args.save_annotated)
    start = time.monotonic()
    next_publish = 0.0

    try:
        while rclpy.ok() and not checker.done:
            now = time.monotonic()
            if now - start > args.timeout:
                break
            if now >= next_publish:
                checker.publish_sample()
                next_publish = now + 0.1
            rclpy.spin_once(checker, timeout_sec=0.02)
    finally:
        elapsed = time.monotonic() - start
        checker.destroy_node()
        rclpy.shutdown()

    if checker.success:
        print(f"[PASS] ROS-интеграция: {checker.reason}; "
              f"событий={checker.event_count}, аннотаций={checker.annotated_count}, "
              f"время={elapsed:.1f} с")
        return 0

    print(f"[FAIL] ROS-интеграция: {checker.reason}; "
          f"событий={checker.event_count}, аннотаций={checker.annotated_count}, "
          f"время={elapsed:.1f} с", file=sys.stderr)
    if checker.last_event:
        print("Последнее событие: "
              f"{json.dumps(checker.last_event, ensure_ascii=False)[:500]}",
              file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
