#!/usr/bin/env python3
"""ROS 2-нода бортового зрения «Око МЧС».

Подписки:
    /mchs/camera/image_raw   sensor_msgs/Image    — кадры с камеры Gazebo
    /mchs/odom               nav_msgs/Odometry    — высота и координаты БПЛА

Публикации:
    /mchs/detection_event    std_msgs/String      — JSON события ЧС
    /mchs/camera/annotated   sensor_msgs/Image    — кадр с тактическим HUD

Запуск:
    source /opt/ros/humble/setup.bash
    python3 src/mchs_vision/scripts/mchs_vision_node.py

Параметры (ROS 2):
    camera_pitch_down_deg=34.4   # текущая камера в mchs_fire_zone.world
    camera_pitch_down_deg=90.0   # строго надирная камера
    fallback_altitude_m=8.0      # если odom.z < min_valid_altitude_m
    show_window:=true            # локальное окно OpenCV (WSLg)
"""

from __future__ import annotations

import json
import math
import os
import sys
from datetime import datetime, timezone
from typing import Optional

import cv2
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

from mchs_vision.config import CameraConfig, DetectorConfig, GeoOrigin
from mchs_vision.events import EventThrottle
from mchs_vision.geometry import Pose
from mchs_vision.pipeline import VisionPipeline


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw из кватерниона (ROS REP-103, вращение вокруг Z)."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class MchsVisionNode(Node):
    def __init__(self) -> None:
        super().__init__("mchs_vision_node")

        self.image_topic = str(self.declare_parameter(
            "image_topic", "/mchs/camera/image_raw").value)
        self.odom_topic = str(self.declare_parameter(
            "odom_topic", "/mchs/odom").value)
        self.event_topic = str(self.declare_parameter(
            "event_topic", "/mchs/detection_event").value)
        self.annotated_topic = str(self.declare_parameter(
            "annotated_topic", "/mchs/camera/annotated").value)

        self.hfov_deg = float(self.declare_parameter(
            "camera_hfov_deg", 80.0).value)
        self.pitch_down_deg = float(self.declare_parameter(
            "camera_pitch_down_deg", 34.4).value)
        self.image_width = int(self.declare_parameter(
            "image_width", 640).value)
        self.image_height = int(self.declare_parameter(
            "image_height", 480).value)
        self.fallback_altitude_m = float(self.declare_parameter(
            "fallback_altitude_m", 8.0).value)
        self.min_valid_altitude_m = float(self.declare_parameter(
            "min_valid_altitude_m", 1.0).value)
        self.max_range_m = float(self.declare_parameter(
            "max_range_m", 300.0).value)

        self.origin_lat = float(self.declare_parameter(
            "origin_lat", 53.4129).value)
        self.origin_lon = float(self.declare_parameter(
            "origin_lon", 58.9814).value)
        self.track_min_hits = int(self.declare_parameter(
            "track_min_hits", 2).value)
        self.event_cooldown_s = float(self.declare_parameter(
            "event_cooldown_s", 3.0).value)
        self.event_min_interval_s = float(self.declare_parameter(
            "event_min_interval_s", 1.0).value)
        self.process_every_n = max(1, int(self.declare_parameter(
            "process_every_n", 1).value))
        self.show_window = bool(self.declare_parameter(
            "show_window", False).value)
        self.publish_annotated = bool(self.declare_parameter(
            "publish_annotated", True).value)
        enabled_raw = str(self.declare_parameter(
            "enabled_classes", "FIRE,SMOKE,OIL_SPILL,FLOOD").value)
        enabled = tuple(name.strip().upper() for name in enabled_raw.split(",")
                        if name.strip())
        title = str(self.declare_parameter(
            "hud_title", "MCHS OPTICAL RECON // GAZEBO 11").value)

        self.bridge = CvBridge()
        self._pose = Pose(x_m=0.0, y_m=0.0, z_m=self.fallback_altitude_m)
        self._pose_received = False
        self._warned_altitude = False
        self._frame_index = 0
        self._logged_events = {}
        self._window_created = False

        self.pipeline = self._build_pipeline(self.image_width,
                                             self.image_height, enabled, title)

        self.create_subscription(Image, self.image_topic, self._image_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback,
                                 10)
        self.event_publisher = self.create_publisher(String, self.event_topic, 10)
        self.annotated_publisher = self.create_publisher(
            Image, self.annotated_topic, 10
        )

        self.get_logger().info(
            ">>> mchs_vision_node запущена: "
            f"image={self.image_topic}, odom={self.odom_topic}, "
            f"event={self.event_topic}, annotated={self.annotated_topic}"
        )
        self.get_logger().info(
            f">>> камера: {self.image_width}x{self.image_height}, "
            f"HFOV={self.hfov_deg}°, наклон={self.pitch_down_deg}°, "
            f"классы={','.join(enabled)}, min_hits={self.track_min_hits}"
        )
        if self.pitch_down_deg >= 89.0:
            self.get_logger().info(">>> камера надирная (pitch=90°)")
        else:
            self.get_logger().info(
                f">>> камера наклонена на {self.pitch_down_deg}° — "
                "геопривязка учитывает перспективу"
            )

    def _build_pipeline(self, width: int, height: int, enabled,
                        title: str) -> VisionPipeline:
        camera = CameraConfig(
            width=max(1, width),
            height=max(1, height),
            hfov_deg=self.hfov_deg,
            pitch_down_deg=self.pitch_down_deg,
            max_range_m=self.max_range_m,
        )
        detector_config = DetectorConfig(enabled=enabled,
                                         track_min_hits=self.track_min_hits)
        return VisionPipeline(
            camera=camera,
            detector_config=detector_config,
            origin=GeoOrigin(lat=self.origin_lat, lon=self.origin_lon),
            title=title,
            throttle=EventThrottle(cooldown_s=self.event_cooldown_s,
                                   min_interval_s=self.event_min_interval_s),
        )

    # ------------------------------------------------------------------
    # Подписки
    # ------------------------------------------------------------------
    def _odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(orientation.x, orientation.y,
                                  orientation.z, orientation.w)
        z = float(position.z)
        if z < self.min_valid_altitude_m:
            if not self._warned_altitude:
                self.get_logger().warn(
                    f"odom.z={z:.2f} м ниже порога {self.min_valid_altitude_m} м; "
                    f"использую высоту {self.fallback_altitude_m} м. "
                    "Проверьте плагин высоты Gazebo."
                )
                self._warned_altitude = True
            z = self.fallback_altitude_m

        self._pose = Pose(
            x_m=float(position.x),
            y_m=float(position.y),
            z_m=z,
            yaw_rad=yaw,
        )
        self._pose_received = True

    def _image_callback(self, msg: Image) -> None:
        self._frame_index += 1
        if self._frame_index % self.process_every_n != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # cv_bridge может кидать разные типы
            self.get_logger().error(f"cv_bridge: {exc}")
            return

        if frame is None or frame.size == 0:
            return

        height, width = frame.shape[:2]
        if (width != self.pipeline.camera.width
                or height != self.pipeline.camera.height):
            self.get_logger().warn(
                f"Размер кадра {width}x{height} не совпал с настройками "
                f"{self.pipeline.camera.width}x{self.pipeline.camera.height}; "
                "перестраиваю проектор."
            )
            self.pipeline = self._build_pipeline(
                width, height, self.pipeline.config.enabled, self.pipeline.title
            )

        if not self._pose_received:
            self.get_logger().warn(
                "Одометрия ещё не получена: использую высоту "
                f"{self.fallback_altitude_m} м и координаты (0, 0).",
                throttle_duration_sec=5.0,
            )

        timestamp = self._stamp_to_datetime(msg)
        draw = self.publish_annotated or self.show_window
        result = self.pipeline.process(frame, self._pose, timestamp, draw=draw)

        for event in result.events:
            self.event_publisher.publish(
                String(data=json.dumps(event, ensure_ascii=False))
            )
            self._log_event(event)

        if self.publish_annotated:
            annotated = self.bridge.cv2_to_imgmsg(result.annotated,
                                                  encoding="bgr8")
            annotated.header = msg.header
            self.annotated_publisher.publish(annotated)

        if self.show_window:
            self._show_window(result.annotated)

    # ------------------------------------------------------------------
    # Вспомогательное
    # ------------------------------------------------------------------
    @staticmethod
    def _stamp_to_datetime(msg: Image) -> datetime:
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return datetime.now().astimezone()
        return datetime.fromtimestamp(
            float(stamp.sec) + float(stamp.nanosec) * 1e-9, tz=timezone.utc
        ).astimezone()

    def _log_event(self, event: dict) -> None:
        incident_id = event.get("incident_id", "")
        threat = event.get("threat_level", "")
        previous = self._logged_events.get(incident_id)
        if previous == threat:
            return
        self._logged_events[incident_id] = threat
        self.get_logger().info(
            f"[{incident_id}] {event.get('hazard_type')} {threat}: "
            f"{event.get('area_sq_m')} м², "
            f"X={event['target_geo']['abs_x']:.1f} "
            f"Y={event['target_geo']['abs_y']:.1f}, "
            f"conf={event.get('confidence')}"
        )

    def _show_window(self, frame) -> None:
        try:
            if not self._window_created:
                cv2.namedWindow("MCHS OPTICAL RECON", cv2.WINDOW_NORMAL)
                self._window_created = True
            cv2.imshow("MCHS OPTICAL RECON", frame)
            cv2.waitKey(1)
        except cv2.error as exc:
            self.get_logger().warn(
                f"Не удалось показать окно OpenCV: {exc}", throttle_duration_sec=10.0
            )
            self.show_window = False

    def destroy_node(self) -> bool:
        if self._window_created:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[MchsVisionNode] = None
    try:
        node = MchsVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
