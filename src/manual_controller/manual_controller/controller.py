import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import pigpio
import time

PWM_PIN_VX = 17     # Flysky Channel 1 (e.g. right joystick left-right)
PWM_PIN_VY = 27     # Flysky Channel 2 (e.g. right joystick up-down)
PWM_PIN_THETA = 22  # Flysky Channel 3 (e.g. left joystick left-right)

PWM_CENTER = 1500  # Neutral
PWM_RANGE = 500    # Range from center to extreme (1000 or 2000)

class Controller(Node):
    def __init__(self):
        super().__init__('Flysky_PWM_Controller')

        # ROS2 publisher
        self.control_publisher = self.create_publisher(Pose2D, 'robot/control_input', 10)

        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Cannot connect to pigpio daemon")

        # Set input mode
        self.pi.set_mode(PWM_PIN_VX, pigpio.INPUT)
        self.pi.set_mode(PWM_PIN_VY, pigpio.INPUT)
        self.pi.set_mode(PWM_PIN_THETA, pigpio.INPUT)

        # Timer to read and publish PWM values
        self.timer = self.create_timer(0.05, self.read_pwm_and_publish)  # 20 Hz

    def normalize_pwm(self, pulse_width):
        # Normalize 1000–2000 μs to -1.0 to 1.0
        return max(-1.0, min(1.0, (pulse_width - PWM_CENTER) / PWM_RANGE))

    def read_pwm_and_publish(self):
        raw_vx = self.pi.get_servo_pulsewidth(PWM_PIN_VX)
        raw_vy = self.pi.get_servo_pulsewidth(PWM_PIN_VY)
        raw_theta = self.pi.get_servo_pulsewidth(PWM_PIN_THETA)

        # Normalize values
        vx = self.normalize_pwm(raw_vx)
        vy = self.normalize_pwm(raw_vy)
        theta = self.normalize_pwm(raw_theta)

        msg = Pose2D()
        msg.x = vx
        msg.y = vy
        msg.theta = theta
        self.control_publisher.publish(msg)

        self.get_logger().info(f"PWM vx={raw_vx} vy={raw_vy} theta={raw_theta} → vx={vx:.2f} vy={vy:.2f} theta={theta:.2f}")

    def destroy_node(self):
        self.pi.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
