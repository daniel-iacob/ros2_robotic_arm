import math

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmSwinger(Node):
    def __init__(self):
        super().__init__("arm_swinger")
        self.publisher_ = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.timer = self.create_timer(0.1, self.move_arm)  # 10Hz update rate
        self.angle = 0.0

    def move_arm(self):
        msg = JointTrajectory()
        msg.joint_names = ["joint_1", "joint_2"]

        point = JointTrajectoryPoint()
        # Sine wave creates a smooth back-and-forth motion
        pos_1 = math.sin(self.angle)
        pos_2 = math.cos(self.angle) * 0.5

        point.positions = [pos_1, pos_2]
        point.time_from_start.nanosec = 100000000  # 0.1 seconds

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.angle += 0.05  # Speed of the swing


def main(args=None):
    rclpy.init(args=args)
    node = ArmSwinger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
