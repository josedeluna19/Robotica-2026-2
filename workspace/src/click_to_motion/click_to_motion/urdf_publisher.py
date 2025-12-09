#!/usr/bin/env python3
"""
urdf_publisher.py

Simple node to publish a URDF string on the topic /<ns>/robot_description
with transient_local durability so RViz (and other late-joiners) receive it.

Usage:
  - Launch one instance per robot namespace (use remapping or ns in launch).
  Example: namespace 'robot1' will publish to '/robot1/robot_description'
"""
from __future__ import annotations
import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class URDFPublisher(Node):
    def __init__(self, urdf_text: str):
        super().__init__('urdf_publisher')
        # Use transient_local so late subscribers (RViz) can get the message
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        # topic depends on namespace; publisher created on 'robot_description' local topic
        topic = 'robot_description'
        self.pub = self.create_publisher(String, topic, qos)
        self.urdf = urdf_text

        # publish once immediately and again after short delay
        self.publish_once()
        self.create_timer(1.0, self.publish_once)  # will keep republishing every 1s (cheap)

        self.get_logger().info(f"{self.get_name()}: publishing URDF on ~/{topic} (transient_local)")

    def publish_once(self):
        msg = String()
        msg.data = self.urdf
        try:
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Publish failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    # Simple CLI: if the process is launched from a launch file, the robot_description is passed as a param.
    # But we'll attempt to fetch robot_description param if available; otherwise read env or file passed via argv.
    node = None
    try:
        # If robot_description parameter was set on this node, get it (launch will pass it)
        node_temp = rclpy.create_node('urdf_publisher_helper')
        try:
            # returns ParameterValue if present; otherwise exception
            pd = node_temp.get_parameter_or('robot_description', None)
            if pd is not None and pd.value is not None:
                urdf_txt = pd.value
            else:
                # fallback: check argv[1] as file path
                if len(sys.argv) > 1 and os.path.exists(sys.argv[1]):
                    with open(sys.argv[1], 'r') as f:
                        urdf_txt = f.read()
                else:
                    urdf_txt = ''
        except Exception:
            urdf_txt = ''
        node_temp.destroy_node()

        if not urdf_txt:
            # try env var
            urdf_txt = os.environ.get('URDF_STRING', '')
        if not urdf_txt:
            # if still empty, exit
            raise RuntimeError("No URDF provided via parameter, argv or URDF_STRING environment variable.")

        node = URDFPublisher(urdf_txt)
        rclpy.spin(node)
    except Exception as e:
        print("urdf_publisher failed to start:", e, file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
