import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import random
import threading
import sys
import termios
import tty
import select

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.timer = self.create_timer(0.1, self.move_turtle)

        self.pose = None
        self.mode = "AUTO"

        # Start keyboard listener thread
        threading.Thread(target=self.key_listener, daemon=True).start()

    def pose_callback(self, msg: Pose):
        self.pose = msg

    def key_listener(self):
        """Listen for SPACE key asynchronously"""
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.get_logger().info("Press SPACE to return to origin (CTRL+C to quit)")
        try:
            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == " ":
                        self.get_logger().info("SPACE pressed → Returning to origin")
                        self.mode = "RETURN"
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def move_turtle(self):
        if self.pose is None:
            return

        cmd = Twist()
        if self.mode == "AUTO":
            margin = 1.0
            if (self.pose.x < margin or self.pose.x > 11.0 - margin or
                self.pose.y < margin or self.pose.y > 11.0 - margin):
                cmd.linear.x = 0.0
                cmd.angular.z = random.choice([-1.5, 1.5])
            else:
                cmd.linear.x = 2.0
                cmd.angular.z = 0.0
        elif self.mode == "RETURN":
            dx = 0.0 - self.pose.x
            dy = 0.0 - self.pose.y
            angle_to_goal = math.atan2(dy, dx)
            angle_diff = angle_to_goal - self.pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            if abs(angle_diff) > 0.1:
                cmd.linear.x = 0.0
                cmd.angular.z = 1.5 if angle_diff > 0 else -1.5
            else:
                distance = math.sqrt(dx**2 + dy**2)
                if distance > 0.2:
                    cmd.linear.x = 2.0
                    cmd.angular.z = 0.0
                else:
                    self.get_logger().info("Reached origin → Switching to AUTO")
                    self.mode = "AUTO"
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
