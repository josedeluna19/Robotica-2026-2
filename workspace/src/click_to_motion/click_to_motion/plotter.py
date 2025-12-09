# src/click_to_motion/click_to_motion/plotter.py
import rclpy
from rclpy.node import Node
from click_to_motion_msgs.msg import JointControl
import matplotlib.pyplot as plt

class Plotter(Node):
    def __init__(self):
        super().__init__('plotter')
        self.declare_parameter('robot_ns','')
        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value.strip('/')
        sub_topic = f"/{self.robot_ns}/joint_target" if self.robot_ns else "/joint_target"
        self.create_subscription(JointControl, sub_topic, self.callback, 10)
        self.get_logger().info(f"Plotter ready for '{self.robot_ns}'")

    def callback(self, msg: JointControl):
        plt.figure(figsize=(4,3))
        plt.plot(msg.q, 'o-')
        plt.title(f"Joint targets ({self.robot_ns or 'root'})")
        plt.xlabel("Joint index")
        plt.ylabel("Position (rad)")
        plt.grid(True)
        # non-blocking draw
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = Plotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

