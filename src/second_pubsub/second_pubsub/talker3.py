#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        # Create publishers for both topics
        self.moving_publisher = self.create_publisher(String, 'robot/moving', 10)
        self.lift_publisher = self.create_publisher(String, 'robot/lift', 10)
        
        # Subscriber to incoming keys
        self.subscription = self.create_subscription(
            String,
            'remote_keys',
            self.key_callback,
            10
        )

        self.get_logger().info('Controller node started. Listening for key commands on /remote_keys')

    def key_callback(self, msg: String):
        key = msg.data.lower().strip()
        if not key:
            return

        if key == 'x':
            self.get_logger().info('Received "x" - shutting down')
            rclpy.shutdown()
            return

        if key in ['w', 'a', 's', 'd', 'c']:
            cmd = String()
            if key == 'w':
                cmd.data = '255 255 255 255'
            elif key == 's':
                cmd.data = '-255 -255 -255 -255'
            elif key == 'a':
                cmd.data = '255 -255 -255 255'
            elif key == 'd':
                cmd.data = '-255 255 255 -255'
            elif key == 'c':
                cmd.data = '0 0 0 0'
            self.moving_publisher.publish(cmd)
            self.get_logger().info(f'Published to robot/moving: "{cmd.data}"')

        elif key in ['q', 'e']:
            cmd = String()
            cmd.data = 'UP' if key == 'q' else 'DOWN'
            self.lift_publisher.publish(cmd)
            self.get_logger().info(f'Published to robot/lift: "{cmd.data}"')


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
