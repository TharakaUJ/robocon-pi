import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

from PID.PID import PIDController  # your singleton class
import struct
import serial
import os
from dotenv import load_dotenv
from geometry_msgs.msg import Pose2D

class PIDCalibrator(Node):
    def __init__(self):
        super().__init__('pid_calibrator')

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


        self.srv = self.create_service(Trigger, 'calibrate_pid', self.calibrate_callback)

       

        self.odometry_subscription = self.create_subscription(
        Pose2D,
        'robot/velocity',
        self.current_velocity_callback,
        10)

        # open serial connection
        self.serial = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)

        self.get_logger().info("PID Calibrator ready.")

    def current_velocity_callback(self, msg: Pose2D):
        self.vx = msg.x
        self.vy = msg.y
        self.theta = msg.theta

    def calibrate_callback(self, request, response):
        self.get_logger().info("Starting PID calibration...")

        # --- Step 1: Send step commands ---
        actual_responses = []
        desired_v = 0.5  # desired linear velocity
        N = 20

        for _ in range(N):
            packet = struct.pack('<fff', desired_v, 0.0, 0.0)
            self.serial.write(packet)
            time.sleep(0.2)
            actual_responses.append(self.vx)

        # --- Step 2: Evaluate response ---
        errors = [desired_v - x for x in actual_responses]
        avg_error = sum(errors) / len(errors)
        error_deriv = [(errors[i] - errors[i - 1]) / 0.2 for i in range(1, len(errors))]
        avg_deriv = sum(error_deriv) / len(error_deriv)

        # --- Step 3: Estimate PID gains (simple heuristic) ---
        kp = avg_error * 2.0
        ki = avg_error * 0.5
        kd = avg_deriv * 0.2

        self.pid_controller.set_kp(kp)
        self.pid_controller.set_ki(ki)
        self.pid_controller.set_kd(kd)
        self.pid_controller._log_params()

        self.get_logger().info(f"New PID gains -> kp: {kp:.2f}, ki: {ki:.2f}, kd: {kd:.2f}")

        response.success = True
        response.message = f"Calibrated: kp={kp:.2f}, ki={ki:.2f}, kd={kd:.2f}"
        return response
