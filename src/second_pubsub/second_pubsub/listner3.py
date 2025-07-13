#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import os

class ESP32(Node):
    def __init__(self):
        super().__init__('ESP32')
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
            self.get_logger().info(f'Successfully connected to ESP32 on {self.serial_port}')
            
        except (serial.SerialException, OSError) as e:
            self.get_logger().warning(f'Failed to connect to {self.serial_port}: {str(e)}')
        
        if not self.serial_connection or not self.serial_connection.is_open:
            self.get_logger().error('Could not connect to ESP32 on any port!')
            raise RuntimeError('ESP32 connection failed')

        # Create subscribers
        self.moving_subscription = self.create_subscription(
            String,
            'robot/moving',
            self.moving_callback,
            10)
        self.lift_subscription = self.create_subscription(
            String,
            'robot/lift',
            self.lift_callback,
            10)
        
        self.get_logger().info('ESP32 node ready. Listening to ROS topics...')
    def moving_callback(self, msg):
        self.get_logger().info(f'Received movement command: "{msg.data}"')
        try:
            # Send command to ESP32 in format: "MOVE:<values>"
            command = f"MOVE:{msg.data}\n"
            self.serial_connection.write(command.encode('utf-8'))
            self.get_logger().debug(f'Sent to ESP32: {command.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error sending movement command: {str(e)}')

    def lift_callback(self, msg):
        self.get_logger().info(f'Received lift command: "{msg.data}"')
        try:
            # Send command to ESP32 in format: "LIFT:<UP/DOWN>"
            command = f"LIFT:{msg.data}\n"
            self.serial_connection.write(command.encode('utf-8'))
            self.get_logger().debug(f'Sent to ESP32: {command.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error sending lift command: {str(e)}')

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
