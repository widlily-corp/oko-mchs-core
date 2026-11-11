#!/usr/bin/env python3
"""ROS 2-нода бортового зрения «Око МЧС».

Подписки:
    /mchs/camera/image_raw   sensor_msgs/Image    — кадры с камеры Gazebo
    /mchs/odom               nav_msgs/Odometry    — высота и координаты БПЛА

Публикации:
    /mchs/detection_event    std_msgs/String      — JSON события ЧС
    /mchs/camera/annotated   sensor_msgs/Image    — кадр с тактическим HUD
    HTTP :8080/stream        MJPEG                — прямой веб-эфир для Windows UI / браузера

Запуск:
    source /opt/ros/humble/setup.bash
    python3 src/mchs_vision/scripts/mchs_vision_node.py
"""

from __future__ import annotations

import os
import sys

# Настройки графической подсистемы WSLg
if not os.environ.get('DISPLAY'):
    os.environ['DISPLAY'] = ':0'
if not os.environ.get('WAYLAND_DISPLAY'):
    os.environ['WAYLAND_DISPLAY'] = 'wayland-0'
if not os.environ.get('XDG_RUNTIME_DIR'):
    os.environ['XDG_RUNTIME_DIR'] = '/mnt/wslg/runtime-dir'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
os.environ['GALLIUM_DRIVER'] = 'llvmpipe'
os.environ['MESA_LOADER_DRIVER_OVERRIDE'] = 'llvmpipe'

import json
import math
import threading
import time
from datetime import datetime, timezone
from http.server import HTTPServer, BaseHTTPRequestHandler
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

from mchs_vision.config import CameraConfig, DetectorConfig, GeoOrigin
from mchs_vision.events import EventThrottle
from mchs_vision.geometry import Pose
from mchs_vision.pipeline import VisionPipeline

STREAM_PORT = 8080
latest_jpeg_frame = None
frame_lock = threading.Lock()


class MchsStreamHandler(BaseHTTPRequestHandler):
    """Веб-сервер MJPEG-трансляции для отображения в Windows АРМ Оператора и веб-браузере."""
    def log_message(self, format, *args):
        pass  # Отключаем спам в консоль

    def do_GET(self):
        global latest_jpeg_frame, frame_lock
        if self.path in ('/', '/index.html'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            html = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>ОКО МЧС — Камера БПЛА</title>
<style>
body { background: #0b0f19; color: #e2e8f0; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; text-align: center; margin: 0; padding: 20px; }
h1 { color: #38bdf8; font-size: 22px; margin-bottom: 6px; letter-spacing: 1px; }
.sub { color: #94a3b8; font-size: 14px; margin-bottom: 20px; }
.frame-box { display: inline-block; padding: 6px; background: #1e293b; border-radius: 12px; box-shadow: 0 10px 25px rgba(0,0,0,0.5); border: 1px solid #334155; }
img { display: block; border-radius: 8px; max-width: 90vw; height: auto; }
.badge { display: inline-block; padding: 4px 12px; margin-top: 14px; background: #065f46; color: #34d399; border-radius: 9999px; font-size: 13px; font-weight: bold; }
</style></head>
<body>
<h1>ОКО МЧС — ПРЯМОЙ ЭФИР С БОРТОВОЙ КАМЕРЫ БПЛА</h1>
<div class="sub">Gazebo 11 3D Simulation &bull; ROS 2 Topic: <code>/mchs/camera/image_raw</code> &bull; OpenCV AI Detection</div>
<div class="frame-box"><img src="/stream" alt="Live Stream" /></div>
<br><div class="badge">&bull; ТАКТИЧЕСКИЙ ЭФИР АКТИВЕН (30 FPS HD)</div>
</body></html>"""
            self.wfile.write(html.encode('utf-8'))
        elif self.path == '/snapshot.jpg':
            with frame_lock:
                frame_data = latest_jpeg_frame
            if frame_data:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(frame_data)))
                self.end_headers()
                self.wfile.write(frame_data)
            else:
                self.send_response(503)
                self.end_headers()
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                with frame_lock:
                    frame_data = latest_jpeg_frame
                if frame_data is not None:
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(frame_data)))
                        self.end_headers()
                        self.wfile.write(frame_data)
                        self.wfile.write(b'\r\n')
                    except Exception:
                        break
                time.sleep(0.033)
        else:
            self.send_response(404)
            self.end_headers()


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
            "camera_pitch_down_deg", 45.0).value)
        self.image_width = int(self.declare_parameter(
            "image_width", 1280).value)
        self.image_height = int(self.declare_parameter(
            "image_height", 720).value)
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
            "show_window", True).value)
        self.publish_annotated = bool(self.declare_parameter(
            "publish_annotated", True).value)
        enabled_raw = str(self.declare_parameter(
            "enabled_classes", "FIRE,SMOKE,OIL_SPILL,FLOOD,WASTE").value)
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

        self.create_subscription(Image, self.image_topic,
                                 self._image_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic,
                                 self._odom_callback,
                                 qos_profile_sensor_data)

        self.event_publisher = self.create_publisher(String, self.event_topic, 10)
        self.annotated_publisher = self.create_publisher(Image, self.annotated_topic, 10)

        # Запуск фонового веб-стримера видеопотока на порту 8080
        self.start_web_streamer()

        self.get_logger().info(
            f"Нода зрения 'Око МЧС' запущена. "
            f"Камера HFOV={self.hfov_deg}°, pitch={self.pitch_down_deg}°. "
            f"Классы: {','.join(enabled)}. "
            f"Окно GUI: {self.show_window}. "
            f"Веб-трансляция: http://localhost:{STREAM_PORT}"
        )

    def start_web_streamer(self):
        try:
            server = HTTPServer(('0.0.0.0', STREAM_PORT), MchsStreamHandler)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
        except Exception as e:
            self.get_logger().warn(f'Не удалось запустить веб-стример на порту {STREAM_PORT}: {e}')

    def _build_pipeline(self, width: int, height: int,
                        enabled: tuple[str, ...],
                        title: str) -> VisionPipeline:
        cam_cfg = CameraConfig(
            width=width,
            height=height,
            hfov_deg=self.hfov_deg,
            pitch_down_deg=self.pitch_down_deg,
            max_range_m=self.max_range_m,
        )
        det_cfg = DetectorConfig(
            enabled=enabled,
            track_min_hits=self.track_min_hits,
            event_cooldown_s=self.event_cooldown_s,
        )
        origin = GeoOrigin(lat_deg=self.origin_lat, lon_deg=self.origin_lon)
        throttle = EventThrottle(min_interval_s=self.event_min_interval_s)
        return VisionPipeline(
            camera=cam_cfg,
            detector=det_cfg,
            origin=origin,
            throttle=throttle,
            hud_title=title,
        )

    def _odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        z = float(position.z)
        if z < self.min_valid_altitude_m:
            if not self._warned_altitude:
                self.get_logger().warn(
                    f"odom.z={z:.2f} м ниже порога {self.min_valid_altitude_m} м; "
                    f"использую высоту {self.fallback_altitude_m} м."
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
        except Exception as exc:
            self.get_logger().error(f"cv_bridge: {exc}")
            return

        if frame is None or frame.size == 0:
            return

        height, width = frame.shape[:2]
        if (width != self.pipeline.camera.width
                or height != self.pipeline.camera.height):
            self.pipeline = self._build_pipeline(
                width, height, self.pipeline.config.enabled, self.pipeline.title
            )

        timestamp = self._stamp_to_datetime(msg)
        draw = True  # Всегда рисуем HUD для трансляции и окна
        result = self.pipeline.process(frame, self._pose, timestamp, draw=draw)

        # Рассылка событий ЧС (для автопилота и ЦУКС)
        for event in result.events:
            self.event_publisher.publish(
                String(data=json.dumps(event, ensure_ascii=False))
            )
            self._log_event(event)

        # Публикация в ROS топик
        if self.publish_annotated and result.annotated is not None:
            annotated = self.bridge.cv2_to_imgmsg(result.annotated, encoding="bgr8")
            annotated.header = msg.header
            self.annotated_publisher.publish(annotated)

        # Обновление веб-трансляции на :8080
        global latest_jpeg_frame, frame_lock
        out_frame = result.annotated if result.annotated is not None else frame
        try:
            ret, jpeg = cv2.imencode('.jpg', out_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret:
                with frame_lock:
                    latest_jpeg_frame = jpeg.tobytes()
        except Exception:
            pass

        # Локальное окно OpenCV (WSLg)
        if self.show_window and result.annotated is not None:
            self._show_window(result.annotated)

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
                cv2.resizeWindow("MCHS OPTICAL RECON", 960, 540)
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
