# src/click_to_motion/click_to_motion/torque_computer.py
import rclpy
from rclpy.node import Node
from click_to_motion_msgs.msg import JointControl

class TorqueComputer(Node):
    def __init__(self):
        super().__init__('torque_computer')
        self.declare_parameter('robot_ns','')
        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value.strip('/')
        sub_topic = f"/{self.robot_ns}/joint_target" if self.robot_ns else "/joint_target"
        self.create_subscription(JointControl, sub_topic, self.callback, 10)
        self.get_logger().info(f"Torque computer started for '{self.robot_ns}'")

    def callback(self, msg: JointControl):
        torques = [0.5 * q for q in msg.q]  # placeholder
        self.get_logger().info(f"[{self.robot_ns}] Torques estimados: {torques}")

def main(args=None):
    rclpy.init(args=args)
    node = TorqueComputer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
