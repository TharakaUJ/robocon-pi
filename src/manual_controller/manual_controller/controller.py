import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import pigpio
import time

PWM_PIN_VX = 17
PWM_PIN_VY = 27
PWM_PIN_THETA = 22

NEUTRAL_PW = 1500  # microseconds
TOLERANCE = 50     # deadzone for neutral
FAILSAFE_TIMEOUT = 0.5  # seconds

def normalize_pwm(pw):
    """Convert PWM to normalized range [-1, 1]"""
    return max(-1.0, min(1.0, (pw - NEUTRAL_PW) / 500.0))  # 1000-2000 -> -1 to 1

class PWMInput:
    def __init__(self, pi, pin):
        self.pi = pi
        self.pin = pin
        self.last_value = NEUTRAL_PW
        self.last_change = time.time()
        self.pi.set_mode(pin, pigpio.INPUT)
        self.pi.set_pull_up_down(pin, pigpio.PUD_OFF)
        self.cb = self.pi.callback(pin, pigpio.EITHER_EDGE, self._cb)
        self._high_tick = None

    def _cb(self, gpio, level, tick):
        if level == 1:
            self._high_tick = tick
        elif level == 0 and self._high_tick is not None:
            pulse_width = pigpio.tickDiff(self._high_tick, tick)
            if 900 <= pulse_width <= 2100:  # valid range
                self.last_value = pulse_width
                self.last_change = time.time()

    def get_value(self):
        return self.last_value, self.last_change

class Controller(Node):
    def __init__(self):
        super().__init__('flysky_controller')
        self.publisher = self.create_publisher(Pose2D, 'robot/control_input', 10)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Failed to connect to pigpio daemon.")
            raise RuntimeError("Cannot connect to pigpio")

        self.input_vx = PWMInput(self.pi, PWM_PIN_VX)
        self.input_vy = PWMInput(self.pi, PWM_PIN_VY)
        self.input_theta = PWMInput(self.pi, PWM_PIN_THETA)

        self.last_publish_time = 0.0
        self.last_values = (0.0, 0.0, 0.0)

        self.timer = self.create_timer(0.02, self.check_and_publish)  # 50Hz

    def check_and_publish(self):
        now = time.time()
        vx_pw, vx_time = self.input_vx.get_value()
        vy_pw, vy_time = self.input_vy.get_value()
        theta_pw, theta_time = self.input_theta.get_value()

        # Fail-safe: if no update in timeout period, skip publish
        if (now - vx_time > FAILSAFE_TIMEOUT or
            now - vy_time > FAILSAFE_TIMEOUT or
            now - theta_time > FAILSAFE_TIMEOUT):
            self.get_logger().warn("No RC signal detected: entering failsafe.")
            return

        # Normalize input to -1.0 to +1.0
        vx = normalize_pwm(vx_pw)
        vy = normalize_pwm(vy_pw)
        theta = normalize_pwm(theta_pw)

        # Apply deadzone
        vx = 0.0 if abs(vx) < 0.05 else vx
        vy = 0.0 if abs(vy) < 0.05 else vy
        theta = 0.0 if abs(theta) < 0.05 else theta

        # Only publish if values changed significantly
        if (vx, vy, theta) != self.last_values:
            self.last_values = (vx, vy, theta)

            msg = Pose2D()
            msg.x = vx
            msg.y = vy
            msg.theta = theta
            self.publisher.publish(msg)
            self.get_logger().info(f"Published control: vx={vx:.2f}, vy={vy:.2f}, theta={theta:.2f}")

    def destroy_node(self):
        self.get_logger().info("Shutting down RC controller node.")
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
