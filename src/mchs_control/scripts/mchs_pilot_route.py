#!/usr/bin/env python3
"""
Автономный полетный контроллер БПЛА МЧС для ROS 2.
Патрулирование большой арены (200×200 м) по маршруту через 4 очага ЧС:
  БАЗА (0,0) → ПОЖАР (60,28) → НЕФТЕБАЗА (-58,30) → ЗАТОПЛЕНИЕ (45,-56)
  → ПОЛИГОН ТБО (68,-72) → БАЗА (0,0) → ...
Управление — кинематический плагин libgazebo_ros_planar_move через /mchs/cmd_vel.
Дрон летит на постоянной высоте 22 м (гравитация отключена в модели).

Навигация использует фактическую одометрию /mchs/odom, а не интеграл команд:
это исключает уход за пределы арены при проскальзывании/задержках плагина.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# (название точки, X, Y, пауза-зависание, c)
WAYPOINTS = [
    # Точка наблюдения ставится за 18 м до очага по курсу +X:
    # камера (наклон 45° вниз, высота 22 м) видит очаг по центру кадра.
    ("НАБЛЮДЕНИЕ: ОЧАГ ПОЖАРА", 42.0, 28.0, 4.0),
    ("НАБЛЮДЕНИЕ: НЕФТЕБАЗА", -76.0, 30.0, 4.0),
    ("НАБЛЮДЕНИЕ: ЗОНА ЗАТОПЛЕНИЯ", 27.0, -56.0, 4.0),
    ("НАБЛЮДЕНИЕ: ПОЛИГОН ТБО", 50.0, -72.0, 4.0),
    ("БАЗА МЧС", 0.0, 0.0, 3.0),
]

CRUISE_SPEED = 14.0      # м/с
MIN_APPROACH_SPEED = 2.5  # м/с, чтобы не проскакивать точку
ARRIVE_RADIUS = 2.0      # м
GEOFENCE_RADIUS = 95.0   # м, арена 200×200
LOG_PERIOD = 3.0         # c


class MchsPilotNode(Node):
    def __init__(self):
        super().__init__("mchs_drone_pilot")
        self.publisher = self.create_publisher(Twist, "/mchs/cmd_vel", 10)
        self.create_subscription(Odometry, "/mchs/odom", self._on_odom, 10)
        self.timer = self.create_timer(0.1, self.flight_loop)
        self.start_time = time.time()
        self.last_tick = time.time()

        # Резервная оценка позиции, если одометрия ещё не пришла.
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_ok = False
        self.wp_index = 0
        self.hover_until = 0.0
        self.last_log = 0.0
        self.last_geofence_log = 0.0
        self.get_logger().info(
            ">>> Автопилот МЧС запущен: маршрут БАЗА → ПОЖАР → НЕФТЕБАЗА → "
            "ЗАТОПЛЕНИЕ → СВАЛКА → БАЗА (скорость %.0f м/с)" % CRUISE_SPEED
        )

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)
        self.odom_ok = True

    def flight_loop(self):
        now = time.time()
        dt = min(now - self.last_tick, 0.5)
        self.last_tick = now
        cmd = Twist()

        # Пауза-зависание над очагом
        if now < self.hover_until:
            self.publisher.publish(cmd)
            return

        x = self.odom_x if self.odom_ok else self.pos_x
        y = self.odom_y if self.odom_ok else self.pos_y
        name, tx, ty, hover = WAYPOINTS[self.wp_index]

        # Аварийный геофенс: не выпускаем борт за пределы арены.
        if math.hypot(x, y) > GEOFENCE_RADIUS:
            name, tx, ty, hover = "ГЕОФЕНС: ВОЗВРАТ НА БАЗУ", 0.0, 0.0, 0.0
            if now - self.last_geofence_log >= 2.0:
                self.last_geofence_log = now
                self.get_logger().warn(
                    f"Геофенс сработал: X={x:.1f} Y={y:.1f} — возврат на базу"
                )

        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)

        if dist <= ARRIVE_RADIUS:
            self.hover_until = now + hover
            self.get_logger().info(
                f"[ФИКСАЦИЯ] {name}: очаг в зоне видимости, зависание {hover:.0f} с"
            )
            self.wp_index = (self.wp_index + 1) % len(WAYPOINTS)
            self.publisher.publish(cmd)
            return

        # Плавное торможение у точки: на 14 м/с проскакивали цель.
        speed = min(CRUISE_SPEED, max(MIN_APPROACH_SPEED, dist * 1.5))
        vx = dx / dist * speed
        vy = dy / dist * speed
        cmd.linear.x = vx
        cmd.linear.y = vy
        self.publisher.publish(cmd)

        if not self.odom_ok:
            self.pos_x += vx * dt
            self.pos_y += vy * dt

        if now - self.last_log >= LOG_PERIOD:
            self.last_log = now
            self.get_logger().info(
                f"[ПОЛЕТ] курс на {name}: X={x:.1f} Y={y:.1f} "
                f"(до точки {dist:.0f} м)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MchsPilotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
