#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class FakeJointStatePublisher(Node):
    def __init__(self):
        super().__init__('fake_joint_state_pub')
        self.declare_parameter('rate', 20.0)
        self.rate = float(self.get_parameter('rate').get_parameter_value().double_value)
        topic = 'joint_states'  # remapped by launch to /robotX/joint_states
        self.pub = self.create_publisher(JointState, topic, 10)
        ns = self.get_namespace().strip('/')
        display = f"/{ns}/{topic}" if ns else f"/{topic}"
        self.get_logger().info(f"{self.get_name()}: publishing initial joint_state to {display}")
        self.timer = self.create_timer(1.0/self.rate, self.timer_callback)

    def timer_callback(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['joint1', 'joint2', 'joint3']
        js.position = [0.0, 0.0, 0.0]
        self.pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = FakeJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
