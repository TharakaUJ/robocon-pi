import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from geometry_msgs.msg import Pose2D
import time

# Define your PWM input pins
PWM_PIN_VX = 17
PWM_PIN_VY = 27
PWM_PIN_THETA = 22

def read_pwm(pin):
    # Dummy implementation: replace with actual PWM reading logic
    # For real PWM reading, you may need to use pigpio or measure pulse width
    return 0.0

class Controller(Node):
    def __init__(self):
        super().__init__('ESP32_ODOMETRY')
        self.control_publisher = self.create_publisher(Pose2D, 'robot/control_input', 10)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PWM_PIN_VX, GPIO.IN)
        GPIO.setup(PWM_PIN_VY, GPIO.IN)
        GPIO.setup(PWM_PIN_THETA, GPIO.IN)
        self.timer = self.create_timer(0.05, self.read_pwm_and_publish)  # 20Hz

    def read_pwm_and_publish(self):
        vx = read_pwm(PWM_PIN_VX)
        vy = read_pwm(PWM_PIN_VY)
        theta = read_pwm(PWM_PIN_THETA)

        # Create and publish the Pose2D message
        msg = Pose2D()
        msg.x = vx
        msg.y = vy
        msg.theta = theta
        self.control_publisher.publish(msg)
        self.get_logger().info(f'Published control input: vx={vx}, vy={vy}, theta={theta}')

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

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
