# src/click_to_motion/click_to_motion/trajectory_generator.py
import rclpy
from rclpy.node import Node
from click_to_motion_msgs.msg import JointControl
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.declare_parameter('robot_ns', '')
        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value.strip('/')
        sub_topic = f"/{self.robot_ns}/joint_target" if self.robot_ns else "/joint_target"
        pub_topic = f"/{self.robot_ns}/trajectory_out" if self.robot_ns else "/trajectory_out"
        self.create_subscription(JointControl, sub_topic, self.callback, 10)
        self.publisher_ = self.create_publisher(JointTrajectory, pub_topic, 10)
        self.get_logger().info(f"Trajectory generator started for '{self.robot_ns}'")

    def callback(self, msg: JointControl):
        traj = JointTrajectory()
        traj.joint_names = ["joint1", "joint2", "joint3"]
        pt = JointTrajectoryPoint()
        pt.positions = list(msg.q)
        # set a time_from_start of 2 seconds
        pt.time_from_start = Duration(sec=2, nanosec=0)
        traj.points.append(pt)
        self.publisher_.publish(traj)
        self.get_logger().info(f"Trajectory published for [{self.robot_ns}] -> {pt.positions}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
