# src/click_to_motion/click_to_motion/ik_solver.py
import rclpy
from rclpy.node import Node
from click_to_motion_msgs.msg import WorkspacePosition, JointControl
import math
import numpy as np

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class IKSolver(Node):
    def __init__(self):
        super().__init__('ik_solver')
        self.declare_parameter('robot_ns', '')
        self.declare_parameter('robot_type', 'planar')  # 'planar' or 'spatial'
        self.declare_parameter('l1', 0.3)
        self.declare_parameter('l2', 0.3)
        self.declare_parameter('l3', 0.0)

        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value.strip('/')
        self.robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        self.l1 = float(self.get_parameter('l1').get_parameter_value().double_value)
        self.l2 = float(self.get_parameter('l2').get_parameter_value().double_value)
        self.l3 = float(self.get_parameter('l3').get_parameter_value().double_value)

        sub_topic = f"/{self.robot_ns}/workspace_target" if self.robot_ns else "/workspace_target"
        pub_topic = f"/{self.robot_ns}/joint_target" if self.robot_ns else "/joint_target"

        self.subscription = self.create_subscription(WorkspacePosition, sub_topic, self.callback, 10)
        self.publisher_ = self.create_publisher(JointControl, pub_topic, 10)
        self.get_logger().info(f"IK Solver activo para '{self.robot_ns}', tipo {self.robot_type}, links {self.l1},{self.l2},{self.l3}")

    def callback(self, msg: WorkspacePosition):
        if self.robot_type == 'planar':
            q = self.ik_planar(msg.x, msg.y)
        else:
            q = self.ik_spatial(msg.x, msg.y, msg.z)
        jc = JointControl()
        jc.q = [float(v) for v in q]
        self.publisher_.publish(jc)
        self.get_logger().info(f"[{self.robot_ns}] Juntas objetivo: {jc.q}")

    def ik_planar(self, x, y):
        # 3-DOF: q1 = base yaw, q2,q3 planar (shoulder/elbow)
        q1 = math.atan2(y, x)
        r = math.hypot(x, y)
        r_wrist = max(1e-6, r - self.l3)  # avoid zero

        l1, l2 = self.l1, self.l2

        # law of cosines for q3
        cos_q3 = clamp((r_wrist**2 - l1**2 - l2**2) / (2 * l1 * l2), -1.0, 1.0)
        q3 = math.acos(cos_q3)

        # compute q2
        k1 = l1 + l2 * math.cos(q3)
        k2 = l2 * math.sin(q3)

        # angle from base to wrist in plane
        phi = 0.0
        if r_wrist > 1e-6:
            phi = math.atan2(0.0, r_wrist)  # since we work in radial coordinate, phi = 0
        # use standard formula: q2 = atan2(y_wrist, x_wrist) - atan2(k2, k1)
        # but since we collapsed plane to radial, approximate:
        gamma = math.atan2(k2, k1)
        # compute beta via law of cosines for triangle (l1, r_wrist, l2)
        cos_beta = clamp((r_wrist**2 + l1**2 - l2**2) / (2 * l1 * r_wrist) if r_wrist != 0 else 1.0, -1.0, 1.0)
        beta = math.acos(cos_beta)
        q2 = beta - gamma

        return [q1, q2, q3]

    def ik_spatial(self, x, y, z):
        # base yaw
        q1 = math.atan2(y, x)
        r = math.hypot(x, y)
        tx = r
        tz = z
        l1, l2, l3 = self.l1, self.l2, self.l3

        def fk_planar(t2, t3):
            px = l2 * math.cos(t2) + l3 * math.cos(t2 + t3)
            pz = l1 + l2 * math.sin(t2) + l3 * math.sin(t2 + t3)
            return px, pz

        best = (None, float('inf'), 0.0, 0.0)
        for a in np.linspace(-math.pi/2, math.pi/2, 30):
            for b in np.linspace(-math.pi, math.pi, 30):
                px, pz = fk_planar(a, b)
                err = (px - tx) ** 2 + (pz - tz) ** 2
                if err < best[1]:
                    best = ((px, pz), err, a, b)
        q2 = best[2]
        q3 = best[3]
        return [q1, q2, q3]

def main(args=None):
    rclpy.init(args=args)
    node = IKSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

