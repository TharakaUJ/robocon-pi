#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Pose2D

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        # Create publishers for both topics
        self.moving_publisher = self.create_publisher(Int32MultiArray, 'robot/moving', 10)
        
        # Subscriber to incoming movements
        self.subscription = self.create_subscription(
            Pose2D,
            'movement',
            self.key_callback,
            10
        )

        self.get_logger().info('Controller node started. Listening for key commands on /movement')

    def key_callback(self, msg: Pose2D):
        vx = int(msg.x)
        vy = int(msg.y)
        orientation = int(msg.theta)

        # Example: Map vx, vy, orientation to motor commands
        # This is a simple placeholder mapping
        cmd = Int32MultiArray()
        cmd.data = [vx, vy, orientation]

        self.moving_publisher.publish(cmd)
        self.get_logger().info(f'Published to robot/moving: "{cmd.data}"')


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
