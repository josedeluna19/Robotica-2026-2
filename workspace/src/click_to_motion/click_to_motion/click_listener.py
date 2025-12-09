# src/click_to_motion/click_to_motion/click_listener.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from click_to_motion_msgs.msg import WorkspacePosition

class ClickListener(Node):
    def __init__(self):
        super().__init__('click_listener')

        self.declare_parameter('robot_ns', '')
        robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value.strip('/')
        self.robot_ns = robot_ns

        topic = f"/{robot_ns}/clicked_point" if robot_ns else "/clicked_point"
        self.get_logger().info(f"ClickListener initializing for namespace '{robot_ns}' listening on {topic}")

        self.subscription = self.create_subscription(PointStamped, topic, self.listener_callback, 10)

        pub_topic = f"/{robot_ns}/workspace_target" if robot_ns else "/workspace_target"
        self.publisher_ = self.create_publisher(WorkspacePosition, pub_topic, 10)

        self.get_logger().info("Click listener activo.")

    def listener_callback(self, msg: PointStamped):
        wp = WorkspacePosition()
        wp.x = msg.point.x
        wp.y = msg.point.y
        wp.z = msg.point.z
        self.publisher_.publish(wp)
        self.get_logger().info(f"Punto recibido [{self.get_name()}] ns='{self.robot_ns}': {wp.x:.3f}, {wp.y:.3f}, {wp.z:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = ClickListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
