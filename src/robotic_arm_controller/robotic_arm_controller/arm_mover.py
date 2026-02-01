import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ArmSwinger(Node):
    def __init__(self):
        super().__init__("arm_swinger")
        
        # Consistent topic naming
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            "/arm_controller/joint_trajectory", 
            10
        )
        
        # Timer: Increase frequency for smoother visuals (20Hz)
        self.timer_period = 0.05 
        self.timer = self.create_timer(self.timer_period, self.move_arm)
        
        self.angle = 0.0
        self.get_logger().info("Robotic Arm Mover has started - Figure-Eight Pattern")

    def move_arm(self):
        msg = JointTrajectory()
        # MUST match the joint names in your new Xacro exactly
        msg.joint_names = ["joint_1", "joint_2", "joint_3"]

        point = JointTrajectoryPoint()

        # --- THE MOTION LOGIC ---
        # Joint 1: Base rotation (Slow side-to-side)
        pos_1 = math.sin(self.angle) * 0.8
        
        # Joint 2: Shoulder (The 'Figure-Eight' vertical component)
        pos_2 = math.sin(2 * self.angle) * 0.4
        
        # Joint 3: Elbow (Counter-swing to keep it looking natural)
        pos_3 = math.cos(self.angle) * 0.5

        point.positions = [pos_1, pos_2, pos_3]

        # --- TIMING ---
        # Setting time_from_start tells the controller how to interpolate
        point.time_from_start = Duration(sec=0, nanosec=int(self.timer_period * 1e9))

        msg.points.append(point)
        self.publisher_.publish(msg)
        
        # Speed of simulation (Lower = Smoother/Slower)
        self.angle += 0.03 

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArmSwinger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Proper cleanup to prevent "Node already exists" errors on restart
        rclpy.shutdown()

if __name__ == "__main__":
    main()