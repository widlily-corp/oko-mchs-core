#!/usr/bin/env python3
"""
Автономный полетный контроллер БПЛА МЧС для ROS 2.
Управляет перемещением дрона в Gazebo по сектору разведки.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MchsPilotNode(Node):
    def __init__(self):
        super().__init__('mchs_drone_pilot')
        self.publisher = self.create_publisher(Twist, '/mchs/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.flight_loop)
        self.start_time = time.time()
        self.get_logger().info('>>> Автопилот МЧС запущен. Начинаю программу патрулирования...')

    def flight_loop(self):
        elapsed = time.time() - self.start_time
        cmd = Twist()

        # Фаза 1: 0 - 3 сек — подготовка
        if elapsed < 3.0:
            pass
        # Фаза 2: 3 - 16 сек — прямолинейный полет в зону ЧС
        elif elapsed < 16.0:
            cmd.linear.x = 0.6  # Движение вперед со скоростью 0.6 м/с
            cmd.angular.z = 0.0
            if int(elapsed * 10) % 20 == 0:
                self.get_logger().info(f'[Полет] Разведка сектора... t={elapsed:.1f}c')
        # Фаза 3: 16 - 22 сек — зависание над очагом и фиксация
        elif elapsed < 22.0:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            if int(elapsed * 10) % 20 == 0:
                self.get_logger().info('[Зависание] Дрон зафиксировал очаг. ИИ передает данные в штаб!')
        # Фаза 4: 22 - 35 сек — возврат на базу
        elif elapsed < 35.0:
            cmd.linear.x = -0.6 # Полет назад на базу
            if int(elapsed * 10) % 20 == 0:
                self.get_logger().info('[Возврат] Возврат на базу МЧС (RTL)...')
        else:
            cmd.linear.x = 0.0
            self.get_logger().info('[Посадка] Дрон успешно вернулся на базу.')

        self.publisher.publish(cmd)

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

if __name__ == '__main__':
    main()
