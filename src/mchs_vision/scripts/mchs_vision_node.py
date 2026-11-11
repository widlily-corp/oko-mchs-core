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
    camera_pitch_down_deg=45.0   # текущая камера mchs_drone/model.sdf
    camera_pitch_down_deg=90.0   # строго надирная камера
    fallback_altitude_m=8.0      # если odom.z < min_valid_altitude_m
    enable_stream:=true          # HTTP-поток для АРМ на :8080
    show_window:=true            # локальное окно OpenCV (WSLg)
"""

from __future__ import annotations

import json
import math
import os
import sys
from datetime import datetime, timezone
from typing import Optional

# WSLg: те же настройки, что в scripts/run_all.sh и старой ноде Руслана.
# Должны быть выставлены ДО импорта cv2, иначе OpenCV выберет не тот бэкенд.
for _env_name, _env_value in (
    ("DISPLAY", ":0"),
    ("WAYLAND_DISPLAY", "wayland-0"),
    ("XDG_RUNTIME_DIR", "/mnt/wslg/runtime-dir"),
    ("LIBGL_ALWAYS_SOFTWARE", "1"),
    ("GALLIUM_DRIVER", "llvmpipe"),
    ("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe"),
):
    os.environ.setdefault(_env_name, _env_value)

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

REPO_ROOT = os.path.dirname(PACKAGE_ROOT)
SRC_DIR = os.path.join(REPO_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

try:
    from mchs_core import get_logger, install_crash_handler, log_event
except ImportError:
    get_logger = None  # type: ignore[assignment]
    install_crash_handler = None  # type: ignore[assignment]
    log_event = None  # type: ignore[assignment]

if install_crash_handler is not None:
    install_crash_handler("VISION")

from mchs_vision.config import CameraConfig, DetectorConfig, GeoOrigin
from mchs_vision.events import EventThrottle
from mchs_vision.geometry import Pose
from mchs_vision.pipeline import VisionPipeline
from mchs_vision.web_stream import LatestFrameStreamer


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw из кватерниона (ROS REP-103, вращение вокруг Z)."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class MchsVisionNode(Node):
    def __init__(self) -> None:
        super().__init__("mchs_vision_node")

        if install_crash_handler is not None:
            install_crash_handler("VISION")
        self.logger = get_logger("mchs_vision", subsystem="VISION") if get_logger is not None else self.get_logger()

        self._fps_start_time: Optional[float] = None
        self._fps_frames_count = 0
        self._current_fps = 0.0

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
            "camera_pitch_down_deg", 45.0).value)
        self.image_width = int(self.declare_parameter(
            "image_width", 1280).value)
        self.image_height = int(self.declare_parameter(
            "image_height", 720).value)
        self.mount_forward_m = float(self.declare_parameter(
            "camera_mount_forward_m", 0.12).value)
        self.mount_up_m = float(self.declare_parameter(
            "camera_mount_up_m", -0.05).value)
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
        self.enable_stream = bool(self.declare_parameter(
            "enable_stream", True).value)
        self.stream_port = int(self.declare_parameter(
            "stream_port", 8080).value)
        self.stream_quality = int(self.declare_parameter(
            "stream_quality", 80).value)
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

        self.streamer: Optional[LatestFrameStreamer] = None
        if self.enable_stream:
            try:
                self.streamer = LatestFrameStreamer(
                    port=self.stream_port, quality=self.stream_quality
                )
                if self.logger:
                    self.logger.info(
                        f"HTTP-видеостример запущен: http://127.0.0.1:{self.stream_port}/ (snapshot: /snapshot.jpg)"
                    )
            except OSError as exc:
                self.get_logger().warn(
                    f"HTTP-поток на порту {self.stream_port} не запущен: {exc}"
                )
                if self.logger:
                    self.logger.error(
                        f"Ошибка запуска HTTP-видеостримера на порту {self.stream_port}: {exc}"
                    )

        self.create_subscription(Image, self.image_topic, self._image_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback,
                                 10)
        self.event_publisher = self.create_publisher(String, self.event_topic, 10)
        self.annotated_publisher = self.create_publisher(
            Image, self.annotated_topic, 10
        )

        init_msg = (
            ">>> mchs_vision_node запущена: "
            f"image={self.image_topic}, odom={self.odom_topic}, "
            f"event={self.event_topic}, annotated={self.annotated_topic}"
        )
        self.get_logger().info(init_msg)
        if self.logger:
            self.logger.info(init_msg)
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
        if self.streamer is not None:
            stream_msg = (
                f">>> HTTP-поток для АРМ: http://127.0.0.1:{self.stream_port}/ "
                f"(snapshot: /snapshot.jpg)"
            )
            self.get_logger().info(stream_msg)
            if self.logger:
                self.logger.info(stream_msg)

    def _build_pipeline(self, width: int, height: int, enabled,
                        title: str) -> VisionPipeline:
        camera = CameraConfig(
            width=max(1, width),
            height=max(1, height),
            hfov_deg=self.hfov_deg,
            pitch_down_deg=self.pitch_down_deg,
            mount_forward_m=self.mount_forward_m,
            mount_up_m=self.mount_up_m,
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
        try:
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            yaw = yaw_from_quaternion(orientation.x, orientation.y,
                                      orientation.z, orientation.w)
            z = float(position.z)
            if z < self.min_valid_altitude_m:
                if not self._warned_altitude:
                    warn_msg = (
                        f"odom.z={z:.2f} м ниже порога {self.min_valid_altitude_m} м; "
                        f"использую высоту {self.fallback_altitude_m} м. "
                        "Проверьте плагин высоты Gazebo."
                    )
                    self.get_logger().warn(warn_msg)
                    if self.logger:
                        self.logger.warning(warn_msg)
                    self._warned_altitude = True
                z = self.fallback_altitude_m

            self._pose = Pose(
                x_m=float(position.x),
                y_m=float(position.y),
                z_m=z,
                yaw_rad=yaw,
            )
            self._pose_received = True
        except Exception as exc:
            self.get_logger().error(f"Ошибка обработки одометрии: {exc}")
            if self.logger:
                self.logger.error(f"Ошибка обработки одометрии /mchs/odom: {exc}")

    def _image_callback(self, msg: Image) -> None:
        self._frame_index += 1
        if self._frame_index % self.process_every_n != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # cv_bridge может кидать разные типы
            self.get_logger().error(f"cv_bridge: {exc}")
            if self.logger:
                self.logger.error(f"Ошибка конвертации cv_bridge: {exc}")
            return

        if frame is None or frame.size == 0:
            return

        # FPS расчет и мониторинг производительности компьютерного зрения
        import time
        now_s = time.time()
        if self._fps_start_time is None:
            self._fps_start_time = now_s
            self._fps_frames_count = 0
        else:
            self._fps_frames_count += 1
            elapsed_fps = now_s - self._fps_start_time
            if elapsed_fps >= 5.0:
                self._current_fps = self._fps_frames_count / elapsed_fps
                if self.logger:
                    self.logger.info(
                        f"Vision FPS: {self._current_fps:.1f} (кадр #{self._frame_index}, "
                        f"{self._fps_frames_count} кадров за {elapsed_fps:.1f} с)"
                    )
                self._fps_start_time = now_s
                self._fps_frames_count = 0

        height, width = frame.shape[:2]
        if (width != self.pipeline.camera.width
                or height != self.pipeline.camera.height):
            dim_warn = (
                f"Размер кадра {width}x{height} не совпал с настройками "
                f"{self.pipeline.camera.width}x{self.pipeline.camera.height}; "
                "перестраиваю проектор."
            )
            self.get_logger().warn(dim_warn)
            if self.logger:
                self.logger.warning(dim_warn)
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
        draw = (self.publish_annotated or self.show_window
                or self.streamer is not None)
        result = self.pipeline.process(frame, self._pose, timestamp, draw=draw)

        for event in result.events:
            self.event_publisher.publish(
                String(data=json.dumps(event, ensure_ascii=False))
            )
            self._log_event(event)
            if log_event is not None:
                lvl = "ERROR" if event.get("threat_level") == "CRITICAL" else "WARNING"
                log_event("VISION", "HAZARD_DETECTED", event, level=lvl)

        if self.streamer is not None:
            self.streamer.update(result.annotated)

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
        msg = (
            f"[{incident_id}] {event.get('hazard_type')} {threat}: "
            f"{event.get('area_sq_m')} м², "
            f"X={event['target_geo']['abs_x']:.1f} "
            f"Y={event['target_geo']['abs_y']:.1f}, "
            f"conf={event.get('confidence')}"
        )
        self.get_logger().info(msg)
        if self.logger:
            if threat == "CRITICAL":
                self.logger.error(msg)
            else:
                self.logger.warning(msg)

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
        if self.logger:
            self.logger.info("Остановка ноды бортового зрения VISION.")
        if self.streamer is not None:
            self.streamer.stop()
            self.streamer = None
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
