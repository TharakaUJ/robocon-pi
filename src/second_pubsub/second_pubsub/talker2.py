#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request, jsonify
import threading

class RobotController(Node):
    def __init__(self):
        super().__init__('flask_controller')
        self.moving_pub = self.create_publisher(String, 'robot/moving', 10)
        self.lift_pub = self.create_publisher(String, 'robot/lift', 10)
        self.get_logger().info('Flask-ROS Controller initialized')

    def handle_command(self, cmd_type, cmd_value):
        msg = String()
        if cmd_type == 'movement':
            if cmd_value == 'forward':
                msg.data = '255 255 255 255'
            elif cmd_value == 'backward':
                msg.data = '-255 -255 -255 -255'
            elif cmd_value == 'left':
                msg.data = '255 -255 -255 255'
            elif cmd_value == 'right':
                msg.data = '-255 255 255 -255'
            else:
                return False
            self.moving_pub.publish(msg)
            return True
        elif cmd_type == 'lift':
            if cmd_value == 'up':
                msg.data = 'UP'
            elif cmd_value == 'down':
                msg.data = 'DOWN'
            else:
                return False
            self.lift_pub.publish(msg)
            return True
        return False

app = Flask(__name__)
controller = None

@app.route('/command', methods=['POST'])
def command_handler():
    if not request.is_json:
        return jsonify({"error": "Request must be JSON"}), 400
    
    data = request.get_json()
    if not all(k in data for k in ('type', 'value')):
        return jsonify({"error": "Missing type or value"}), 400

    success = controller.handle_command(data['type'], data['value'])
    if not success:
        return jsonify({"error": "Invalid command"}), 400
        
    return jsonify({"status": "success", "command": data})

def ros_spin():
    rclpy.spin(controller)

def main(args=None):
    global controller
    rclpy.init(args=args)
    controller = RobotController()

    # Start ROS in background thread
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.daemon = True
    ros_thread.start()

    # Start Flask
    app.run(host='0.0.0.0', port=8888, threaded=True, use_reloader=False)

    # Cleanup
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
