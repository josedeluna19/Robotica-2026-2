import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
import time

L1 = 0.05
L2 = 0.20
L3 = 0.35

class SpatialIK(Node):

    def __init__(self):
        super().__init__('spatial_ik')

        self.create_subscription(PointStamped, '/clicked_point', self.clicked_cb, 10)
        self.pub_joints = self.create_publisher(JointState, 'joint_states', 10)

        self.traj_xyz = []
        self.traj_q = []

    def clicked_cb(self, msg):

        target = np.array([msg.point.x, msg.point.y, msg.point.z])
        q = np.array([0.0, 0.0, 0.0])  # q1, q2, q3

        self.get_logger().info(f"IK objetivo: {target}")

        # IK iterativa
        q_sol = self.solve_ik(q, target)

        if q_sol is None:
            self.get_logger().warn("No converge IK")
            return

        q1, q2, q3 = q_sol

        js = JointState()
        js.name = ["joint1", "joint2", "joint3"]
        js.position = [q1, q2, q3]
        js.header.stamp = self.get_clock().now().to_msg()
        self.pub_joints.publish(js)

        self.traj_xyz.append(target)
        self.traj_q.append([q1, q2, q3])

        self.plot_data()

    def forward_kinematics(self, q):
        q1, q2, q3 = q

        x = np.cos(q1) * (L2*np.cos(q2) + L3*np.cos(q2+q3))
        y = np.sin(q1) * (L2*np.cos(q2) + L3*np.cos(q2+q3))
        z = L1 + L2*np.sin(q2) + L3*np.sin(q2+q3)

        return np.array([x, y, z])

    def jacobian(self, q):
        q1, q2, q3 = q

        J = np.zeros((3,3))

        r = (L2*np.cos(q2) + L3*np.cos(q2+q3))
        J[0,0] = -np.sin(q1)*r
        J[1,0] =  np.cos(q1)*r

        J[2,1] = L2*np.cos(q2)  + L3*np.cos(q2+q3)
        J[2,2] = L3*np.cos(q2+q3)

        return J

    def solve_ik(self, q0, target):

        q = q0.copy()
        alpha = 0.1  # paso
        for _ in range(300):
            pos = self.forward_kinematics(q)
            error = target - pos

            if np.linalg.norm(error) < 0.005:
                return q

            J = self.jacobian(q)
            dq = alpha * J.T @ error
            q += dq

        return None

    def plot_data(self):
        plt.figure(figsize=(10,5))
        xs = [p[0] for p in self.traj_xyz]
        ys = [p[1] for p in self.traj_xyz]
        zs = [p[2] for p in self.traj_xyz]

        plt.subplot(1,2,1)
        plt.plot(xs, ys, 'o-')
        plt.title("XY workspace")

        plt.subplot(1,2,2)
        q1 = [q[0] for q in self.traj_q]
        q2 = [q[1] for q in self.traj_q]
        q3 = [q[2] for q in self.traj_q]
        plt.plot(q1, label="q1")
        plt.plot(q2, label="q2")
        plt.plot(q3, label="q3")
        plt.legend()
        plt.title("Joint trajectories")

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = SpatialIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

