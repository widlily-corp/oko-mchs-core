import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

def main():
    rclpy.init()
    node = Node('test_cam_subscriber')
    received = [False]

    def callback(msg):
        received[0] = True
        node.get_logger().info(f"SUCCESS: Received image {msg.width}x{msg.height}, encoding={msg.encoding}!")

    sub = node.create_subscription(Image, '/mchs/camera/image_raw', callback, 10)
    node.get_logger().info("Waiting up to 5s for image on /mchs/camera/image_raw...")

    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if received[0]:
            break

    node.destroy_node()
    rclpy.shutdown()
    if not received[0]:
        print("TIMEOUT: No image received!")
        exit(1)

if __name__ == '__main__':
    main()
