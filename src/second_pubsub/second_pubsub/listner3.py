#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
import os
from dotenv import load_dotenv
from geometry_msgs.msg import Pose2D
import struct
from second_pubsub.PID.PID import PIDController
class ESP32(Node):
    def __init__(self):
        super().__init__('ESP32')

        self.vx = 0.0
        self.vy = 0.0
        self.theta = 0.0

        self.error_vx = 0.0
        self.error_vy = 0.0
        self.error_theta = 0.0

        # Initialize PID controller
        self.pid_controller = PIDController()

        # Load environment variables
        dotenv_path = os.path.join(os.path.dirname(__file__), '../../../../../../devices.env')
        print(f"Trying to load: {os.path.abspath(dotenv_path)}")
        load_dotenv(dotenv_path)

        self.serial_port = os.getenv("MOVEMENT_PORT")
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
                self.get_logger().info(f'Successfully connected to ESP32-move on {self.serial_port}')
            
        except (serial.SerialException, OSError) as e:
            self.get_logger().warning(f'Failed to connect to {self.serial_port}: {str(e)}')
        
        if not self.serial_connection or not self.serial_connection.is_open:
            self.get_logger().error('Could not connect to ESP32 on any port!')
            raise RuntimeError('ESP32 connection failed')

        # Create subscribers
        self.moving_subscription = self.create_subscription(
            Pose2D,
            'robot/moving',
            self.moving_callback,
            10)
        
        self.odometry_subscription = self.create_subscription(
            Pose2D,
            'robot/velocity',
            self.pid_callback,
            10)
        
        self.get_logger().info('ESP32 node ready. Listening to ROS topics...')

    
    def pid_callback(self, msg: Pose2D):
        # Expecting Pose2D message with vx, vy, theta
        if not isinstance(msg, Pose2D):
            self.get_logger().error('Received message is not of type Pose2D')
            return
        self.vx = msg.x
        self.vy = msg.y
        self.theta = msg.theta
        self.get_logger().info(f'Received velocity command: vx={self.vx}, vy={self.vy}, theta={self.theta}')



    def moving_callback(self, msg):
        # Expecting Pose2D message with vx, vy, theta
        if not isinstance(msg, Pose2D):
            self.get_logger().error('Received message is not of type Pose2D')
            return
        vx = msg.x
        vy = msg.y
        theta = msg.theta
        self.get_logger().info(f'Received movement command: vx={vx}, vy={vy}, theta={theta}')


        # Compute necessary velocities with control loop
        error_vx = vx - self.vx
        error_vy = vy - self.vy
        error_theta = theta - self.theta

        # Here you would implement your PID control logic
        ux = self.vx + error_vx * self.pid_controller.kpx + (error_vx-self.error_vx) * self.pid_controller.kdx
        uy = self.vy + error_vy * self.pid_controller.kpy + (error_vy-self.error_vy)*self.pid_controller.kdy
        utheta = self.theta + error_theta * self.pid_controller.kpt + (error_theta-self.error_theta)*self.pid_controller.kdt

        self.error_vx = error_vx
        self.error_vy = error_vy
        self.error_theta = error_theta


        try:
            # Send command to ESP32 as a structured binary packet: [vx(float32), vy(float32), theta(float32)]
            packet = struct.pack('<fff', ux, uy, utheta)  # Little-endian, 3 floats
            self.serial_connection.write(packet)
            self.get_logger().debug(f'Sent to ESP32 (binary): vx={vx}, vy={vy}, theta={theta}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error sending movement command: {str(e)}')

    def __del__(self):
        if hasattr(self, 'serial_connection') and self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            self.get_logger().info('Serial connection closed')

def main(args=None):
    rclpy.init(args=args)
    try:
        esp32 = ESP32()
        rclpy.spin(esp32)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if 'esp32' in locals():
            esp32.get_logger().error(f'Node error: {str(e)}')
    finally:
        if 'esp32' in locals():
            esp32.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
