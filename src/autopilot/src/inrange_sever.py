from geometry_msgs.msg import Pose
from autopilot.action import Inrange
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import time
import math

class InrangeServer(Node):
    def __init__(self):
        super().__init__('navigate_action_server')

        self._action_server = ActionServer(
            self,
            Inrange,
            'navigate_to_point',
            self.execute_callback)

        self.pose_sub = self.create_subscription(
            Pose,
            '/odometry/pose',
            self.odom_callback,
            10)

        self.current_pose = None

    def odom_callback(self, msg: Pose):
        self.current_pose = msg

    async def execute_callback(self, goal_handle):
        target_x = goal_handle.request.x
        target_y = goal_handle.request.y

        self.get_logger().info(f"New goal: ({target_x}, {target_y})")

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                return Inrange.Result(success=False)

            if self.current_pose is None:
                self.get_logger().info("Waiting for odometry...")
                time.sleep(0.1)
                continue

            dx = target_x - self.current_pose.position.x
            dy = target_y - self.current_pose.position.y
            distance = math.hypot(dx, dy)

            # Feedback
            feedback = Inrange.Feedback()
            feedback.distance_remaining = distance
            goal_handle.publish_feedback(feedback)

            if distance < 0.1:
                self.get_logger().info("Goal reached")
                goal_handle.succeed()
                return Inrange.Result(success=True)

            # Send movement command
            self.send_control_command(dx, dy, distance)

            time.sleep(0.1)

    def send_control_command(self, dx, dy, distance):
        # Normalize direction
        vx = dx / distance * 0.5  # 0.5 m/s max speed
        vy = dy / distance * 0.5

        command = f"FINE_MOVE {vx:.2f} {vy:.2f}"
        self.get_logger().info(f"CMD: {command}")
        # Send via serial, or publish to robot
        # self.serial.write((command + "\n").encode())


def main(args=None):
    rclpy.init(args=args)
    node = InrangeServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
