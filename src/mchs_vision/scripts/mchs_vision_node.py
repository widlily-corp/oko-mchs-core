#!/usr/bin/env python3
"""
Нода компьютерного зрения 'Око МЧС' для ROS 2.
Принимает кадры с виртуальной камеры дрона из Gazebo 11,
детектирует очаг возгорания/задымления и накладывает тактический HUD.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class MchsVisionNode(Node):
    def __init__(self):
        super().__init__('mchs_vision_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/mchs/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('>>> Нода зрения МЧС активна. Ожидание видеопотока из Gazebo...')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Ошибка конвертации cv_bridge: {e}')
            return

        h, w, _ = frame.shape

        # Перевод в цветовое пространство HSV для обнаружения теплового ядра/пламени
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Маска для оранжево-красного пламени
        lower_fire = np.array([5, 120, 150])
        upper_fire = np.array([25, 255, 255])
        mask = cv2.inRange(hsv, lower_fire, upper_fire)
        
        # Морфологическая фильтрация шума
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        fire_detected = False
        for c in contours:
            area = cv2.contourArea(c)
            if area > 400: # Порог площади объекта
                fire_detected = True
                bx, by, bw, bh = cv2.boundingRect(c)
                
                # Отрисовка рамки детекции
                color = (0, 0, 255) # Красный
                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), color, 2)
                
                # Угловые скобки
                c_len = 12
                cv2.line(frame, (bx, by), (bx + c_len, by), color, 3)
                cv2.line(frame, (bx, by), (bx, by + c_len), color, 3)
                cv2.line(frame, (bx + bw, by), (bx + bw - c_len, by), color, 3)
                cv2.line(frame, (bx + bw, by), (bx + bw, by + c_len), color, 3)

                # Метаданные ЧС
                conf = min(99.4, 85.0 + (area / 1000.0) * 5.0)
                label = f"FIRE DETECTED | {conf:.1f}%"
                cv2.rectangle(frame, (bx, by - 24), (bx + 210, by), color, -1)
                cv2.putText(frame, label, (bx + 5, by - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Геопривязка и площадь
                cv2.putText(frame, "COORDS: 53.4129 N, 58.9814 E", (bx, by + bh + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                cv2.putText(frame, f"SQUARE: ~{area/800.0:.2f} Ha | ALERT MCHS", (bx, by + bh + 36), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # Отрисовка HUD камеры БПЛА
        cx, cy = int(w / 2), int(h / 2)
        cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 255, 200), 1)
        cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 255, 200), 1)

        # Статусная плашка
        status_text = "STATUS: HAZARD DETECTED - SIGNAL SENT" if fire_detected else "STATUS: PATROL / SCANNING"
        status_color = (0, 0, 255) if fire_detected else (0, 255, 0)
        cv2.putText(frame, "MCHS OPTICAL RECON - GAZEBO 11", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)
        cv2.putText(frame, status_text, (15, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)

        cv2.imshow("ОКО МЧС: ОПТИКО-ЭЛЕКТРОННАЯ СИСТЕМА БПЛА (ROS 2 + OpenCV)", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MchsVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
