#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from std_msgs.msg import String
import time
import os
import serial
import struct
from geometry_msgs.msg import Pose2D
from dotenv import load_dotenv

class Controller(Node):
    def __init__(self):
        super().__init__('ESP32_ODOMETRY')

        dotenv_path = os.path.join(os.path.dirname(__file__), '../../../../../../devices.env')
        print(f"Trying to load: {os.path.abspath(dotenv_path)}")
        load_dotenv(dotenv_path)


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
            if self.serial_connection and self.serial_connection.is_open:
                self.get_logger().info(f'Successfully connected to ESP32-Odometry on {self.serial_port}')
            
        except (serial.SerialException, OSError) as e:
            self.get_logger().warning(f'Failed to connect to {self.serial_port}: {str(e)}')
        
        if not self.serial_connection or not self.serial_connection.is_open:
            self.get_logger().error('Could not connect to ESP32 on any port!')
            raise RuntimeError('ESP32-Odometry connection failed')


        # Create publishers for topics
        self.odometry_publisher = self.create_publisher(Pose2D, 'robot/odometry', 10)
        self.velocity_publisher = self.create_publisher(Pose2D, 'robot/velocity', 10)
        

        self.get_logger().info('Controller node started. Listening serial connection and publishing to robot/odometry')
        self.timer = self.create_timer(0.05, self.read_serial_line)  # 20Hz

    def read_serial_line(self):
        if self.serial_connection.in_waiting > 0:
            try:
                # Read raw bytes for OdometryData struct: float theta, float x, float y
                struct_fmt = '<6f'  # little-endian, 6 floats
                struct_size = struct.calcsize(struct_fmt)
                raw_bytes = self.serial_connection.read(struct_size)
                if len(raw_bytes) != struct_size:
                    raise ValueError("Incomplete OdometryData struct received")

                theta, x, y, vx, vy, omega = struct.unpack(struct_fmt, raw_bytes)

                pose_msg = Pose2D()
                pose_msg.x = x
                pose_msg.y = y
                pose_msg.theta = theta

                self.odometry_publisher.publish(pose_msg)


                # For velocity, we can use the same values or compute them
                velocity_msg = Pose2D()
                velocity_msg.x = vx
                velocity_msg.y = vy
                velocity_msg.theta = omega

                self.velocity_publisher.publish(velocity_msg)

            except Exception as e:
                self.get_logger().warning(f'Failed to parse odometry data: {e}')


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
