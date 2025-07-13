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
        self.timer = self.create_timer(0.05, self.read_serial_line)  # 20Hz

    def read_serial_line(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                msg = String()
                msg.data = line
                self.publisher.publish(msg)
                self.get_logger().info(f'Published: {line}')


    def __del__(self):
        if hasattr(self, 'serial_connection') and self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            self.get_logger().info('Serial connection closed')

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
