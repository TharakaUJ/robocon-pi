#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import os

class Controller(Node):
    def __init__(self):
        super().__init__('ESP32_ODOMETRY')
        self.serial_port = os.getenv("ODOMETRY_PORT")
        self.baud_rate = 115200
        self.serial_connection = None
        
        try:
            self.get_logger().info(f'Attempting connection to {self.serial_port}...')
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1
            )
            time.sleep(2)  # Wait for connection to establish
            self.get_logger().info(f'Successfully connected to ESP32-Odometry on {self.serial_port}')
            
        except (serial.SerialException, OSError) as e:
            self.get_logger().warning(f'Failed to connect to {self.serial_port}: {str(e)}')
        
        if not self.serial_connection or not self.serial_connection.is_open:
            self.get_logger().error('Could not connect to ESP32 on any port!')
            raise RuntimeError('ESP32-Odometry connection failed')


        # Create publishers for topics
        self.odometry_publisher = self.create_publisher(String, 'robot/odometry', 10)
        

        self.get_logger().info('Controller node started. Listening serial connection and publishing to robot/odometry')

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
