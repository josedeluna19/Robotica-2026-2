#!/usr/bin/env python3
"""
click_to_motion_spatial
Nodo para robot 3-DOF espacial:
- escucha /<ns>/clicked_point (geometry_msgs/PointStamped)
- calcula IK aproximada (q1 yaw + planar for q2,q3)
- genera trayectoria y publica JointState en /<ns>/joint_states
- grafica workspace/joint-space/torques
"""
from __future__ import annotations
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
import math
import time
import threading
import numpy as np
import matplotlib.pyplot as plt

plt.ion()

class SpatialClickToMotion(Node):
    def __init__(self):
        super().__init__('click_to_motion_spatial')
        self.declare_parameter('l1', 0.05)
        self.declare_parameter('l2', 0.4)
        self.declare_parameter('l3', 0.3)
        self.declare_parameter('exec_rate', 50.0)
        self.declare_parameter('traj_time', 2.0)

        ns = self.get_namespace().strip('/')
        self.ns = ns

        self.l1 = float(self.get_parameter('l1').get_parameter_value().double_value)
        self.l2 = float(self.get_parameter('l2').get_parameter_value().double_value)
        self.l3 = float(self.get_parameter('l3').get_parameter_value().double_value)
        self.exec_rate = float(self.get_parameter('exec_rate').get_parameter_value().double_value)
        self.traj_time = float(self.get_parameter('traj_time').get_parameter_value().double_value)

        click_topic = f"/{self.ns}/clicked_point" if self.ns else "/clicked_point"
        joint_topic = f"/{self.ns}/joint_states" if self.ns else "/joint_states"

        self.get_logger().info(f"[{self.ns}] spatial init l1={self.l1},l2={self.l2},l3={self.l3}")
        self.get_logger().info(f"[{self.ns}] click:{click_topic} joint out:{joint_topic}")

        self.click_sub = self.create_subscription(PointStamped, click_topic, self.clicked_cb, 10)
        self.joint_pub = self.create_publisher(JointState, joint_topic, 10)

        self.latest_joint = [0.0, 0.0, 0.0]
        self.have_joint = False
        self.create_subscription(JointState, joint_topic, self.joint_state_cb, 10)

        self.exec_lock = threading.Lock()

    def joint_state_cb(self, msg: JointState):
        if len(msg.position) >= 3:
            self.latest_joint = list(msg.position[:3])
            self.have_joint = True

    def clicked_cb(self, msg: PointStamped):
        x = float(msg.point.x); y = float(msg.point.y); z = float(msg.point.z)
        self.get_logger().info(f"[{self.ns}] Click at ({x:.3f}, {y:.3f}, {z:.3f})")
        q_goal = self.ik_spatial(x, y, z)
        if q_goal is None:
            self.get_logger().warning(f"[{self.ns}] IK failed")
            return
        self.get_logger().info(f"[{self.ns}] IK goal: {q_goal}")

        q_start = self.latest_joint.copy() if self.have_joint else [0.0, 0.0, 0.0]
        t_samples, q_traj = self.generate_trajectory(q_start, q_goal, self.traj_time, int(self.exec_rate * self.traj_time))
        ee_path = [self.fk(q) for q in q_traj]
        torques = [[0.5 * qi for qi in q] for q in q_traj]

        threading.Thread(target=self.plot_all, args=(ee_path, q_traj, torques), daemon=True).start()

        if self.exec_lock.locked():
            self.get_logger().warning(f"[{self.ns}] busy")
            return
        threading.Thread(target=self.execute_trajectory, args=(t_samples, q_traj), daemon=True).start()

    def ik_spatial(self, x, y, z):
        q1 = math.atan2(y, x)
        r = math.hypot(x, y)
        tx, tz = r, z
        l1, l2, l3 = self.l1, self.l2, self.l3

        def fk_plane(a, b):
            px = l2 * math.cos(a) + l3 * math.cos(a + b)
            pz = l1 + l2 * math.sin(a) + l3 * math.sin(a + b)
            return px, pz

        best = (None, 1e9, 0.0, 0.0)
        for a in np.linspace(-math.pi/2, math.pi/2, 36):
            for b in np.linspace(-math.pi, math.pi, 36):
                px, pz = fk_plane(a, b)
                err = (px - tx)**2 + (pz - tz)**2
                if err < best[1]:
                    best = ((px, pz), err, a, b)
        if best[0] is None:
            return None
        q2 = best[2]; q3 = best[3]
        return [self.norm(q1), self.norm(q2), self.norm(q3)]

    def generate_trajectory(self, q0, qf, total_time, N):
        t = np.linspace(0.0, total_time, max(2, N))
        q0 = np.array(q0); qf = np.array(qf)
        q_traj = [list(q0 + (qf - q0) * (tt / total_time)) for tt in t]
        return t, q_traj

    def fk(self, q):
        q1, q2, q3 = q
        a1, a2, a3 = self.l1, self.l2, self.l3
        planar_x = a2 * math.cos(q2) + a3 * math.cos(q2 + q3)
        planar_z = a1 + a2 * math.sin(q2) + a3 * math.sin(q2 + q3)
        x = planar_x * math.cos(q1); y = planar_x * math.sin(q1); z = planar_z
        return (x, y, z)

    def plot_all(self, ee_path, q_traj, torques):
        try:
            fig1 = plt.figure()
            if len(ee_path[0]) == 3:
                xs = [p[0] for p in ee_path]; zs = [p[2] for p in ee_path]
                plt.plot(xs, zs, 'o-'); plt.title(f"{self.ns} EE X-Z"); plt.grid(True)
            else:
                xs = [p[0] for p in ee_path]; ys = [p[1] for p in ee_path]
                plt.plot(xs, ys, 'o-')

            fig2 = plt.figure()
            q_arr = np.array(q_traj); t = np.linspace(0, self.traj_time, q_arr.shape[0])
            plt.plot(t, q_arr[:,0], label='q1'); plt.plot(t, q_arr[:,1], label='q2'); plt.plot(t, q_arr[:,2], label='q3')
            plt.title(f"{self.ns} joints"); plt.legend(); plt.grid(True)

            fig3 = plt.figure()
            torque_arr = np.array(torques)
            plt.plot(t, torque_arr[:,0], label='tau1'); plt.plot(t, torque_arr[:,1], label='tau2'); plt.legend(); plt.grid(True)

            plt.show(); plt.pause(0.001)
        except Exception as e:
            self.get_logger().error(f"Plot error: {e}")

    def execute_trajectory(self, t_samples, q_traj):
        with self.exec_lock:
            js = JointState()
            js.name = ['joint1', 'joint2', 'joint3']
            period = 1.0 / self.exec_rate
            for q in q_traj:
                js.header.stamp = self.get_clock().now().to_msg()
                js.position = list(q)
                self.joint_pub.publish(js)
                time.sleep(period)
            self.get_logger().info(f"[{self.ns}] execution finished")

    def norm(self, a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    node = SpatialClickToMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
