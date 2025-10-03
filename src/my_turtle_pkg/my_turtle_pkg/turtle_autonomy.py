import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
import select
import termios
import math


class TurtleAutonomy(Node):
    def __init__(self):
        super().__init__('turtle_autonomy')

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.pose = Pose()
        self.timer = self.create_timer(0.1, self.control_loop)

        self.returning_to_origin = False
        # Now origin is (0,0)
        self.origin = (0.0, 0.0)

        # Enable keyboard input
        self.settings = termios.tcgetattr(sys.stdin)

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        twist = Twist()

        if self.returning_to_origin:
            # Distance to origin
            dx = self.origin[0] - self.pose.x
            dy = self.origin[1] - self.pose.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance > 0.1:  # Keep moving until close enough
                angle_to_goal = math.atan2(dy, dx)
                angle_diff = angle_to_goal - self.pose.theta

                # Normalize angle to [-pi, pi]
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

                twist.linear.x = 1.5 * distance
                twist.angular.z = 6.0 * angle_diff
            else:
                # Stop when reached
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.returning_to_origin = False
                self.get_logger().info("✅ Reached origin (0,0), resuming wandering...")

        else:
            # Wander: simple forward motion
            twist.linear.x = 2.0
            twist.angular.z = 0.0

            # Turn when close to walls
            if self.pose.x < 2.0 or self.pose.x > 9.0 or self.pose.y < 2.0 or self.pose.y > 9.0:
                twist.angular.z = 2.0

        self.publisher_.publish(twist)

        # Check keyboard input
        if self.is_key_pressed():
            key = sys.stdin.read(1)
            if key == ' ':
                self.returning_to_origin = True
                self.get_logger().info("⏪ Space pressed → Returning to origin (0,0)!")

    def is_key_pressed(self):
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        return dr != []

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleAutonomy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
