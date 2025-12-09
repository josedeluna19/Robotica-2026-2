#!/usr/bin/env python3
"""
click_to_motion_planar
Nodo para robot 3-DOF planar:
- escucha /<ns>/clicked_point (geometry_msgs/PointStamped)
- calcula IK planar
- genera trayectoria en q
- grafica workspace, joint-space, torques (no bloqueante)
- publica JointState en /<ns>/joint_states
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

class PlanarClickToMotion(Node):
    def __init__(self):
        super().__init__('click_to_motion_planar')
        # params
        self.declare_parameter('l1', 0.4)
        self.declare_parameter('l2', 0.35)
        self.declare_parameter('l3', 0.35)
        self.declare_parameter('exec_rate', 50.0)
        self.declare_parameter('traj_time', 2.0)

        # namespace / detection
        ns = self.get_namespace().strip('/')
        self.ns = ns

        self.l1 = float(self.get_parameter('l1').get_parameter_value().double_value)
        self.l2 = float(self.get_parameter('l2').get_parameter_value().double_value)
        self.l3 = float(self.get_parameter('l3').get_parameter_value().double_value)
        self.exec_rate = float(self.get_parameter('exec_rate').get_parameter_value().double_value)
        self.traj_time = float(self.get_parameter('traj_time').get_parameter_value().double_value)

        click_topic = f"/{self.ns}/clicked_point" if self.ns else "/clicked_point"
        joint_topic = f"/{self.ns}/joint_states" if self.ns else "/joint_states"

        self.get_logger().info(f"[{self.ns}] planar node init: l1={self.l1}, l2={self.l2}, l3={self.l3}")
        self.get_logger().info(f"[{self.ns}] subscribing {click_topic} -> publishing {joint_topic}")

        self.click_sub = self.create_subscription(PointStamped, click_topic, self.clicked_cb, 10)
        self.joint_pub = self.create_publisher(JointState, joint_topic, 10)

        # track latest joint state if anyone publishes it (optional)
        self.latest_joint = [0.0, 0.0, 0.0]
        self.have_joint = False
        self.create_subscription(JointState, joint_topic, self.joint_state_cb, 10)

        self.exec_lock = threading.Lock()

    def joint_state_cb(self, msg: JointState):
        if len(msg.position) >= 3:
            self.latest_joint = list(msg.position[:3])
            self.have_joint = True

    def clicked_cb(self, msg: PointStamped):
        x = float(msg.point.x); y = float(msg.point.y)
        self.get_logger().info(f"[{self.ns}] Click at ({x:.3f}, {y:.3f})")

        q_goal = self.ik_planar(x, y)
        if q_goal is None:
            self.get_logger().warning(f"[{self.ns}] IK failed: unreachable")
            return
        self.get_logger().info(f"[{self.ns}] IK goal: {q_goal}")

        q_start = self.latest_joint.copy() if self.have_joint else [0.0, 0.0, 0.0]

        t_samples, q_traj = self.generate_trajectory(q_start, q_goal, self.traj_time, int(self.exec_rate * self.traj_time))
        ee_path = [self.fk(q) for q in q_traj]
        torques = [[0.5 * qi for qi in q] for q in q_traj]

        threading.Thread(target=self.plot_all, args=(ee_path, q_traj, torques), daemon=True).start()

        if self.exec_lock.locked():
            self.get_logger().warning(f"[{self.ns}] execution in progress, skipping")
            return
        threading.Thread(target=self.execute_trajectory, args=(t_samples, q_traj), daemon=True).start()

    def ik_planar(self, x, y):
        r = math.hypot(x, y)
        r_wrist = max(0.0, r - self.l3)
        l1, l2 = self.l1, self.l2
        # reachability
        cos_q2 = (r_wrist**2 - l1**2 - l2**2) / (2.0 * l1 * l2)
        if cos_q2 < -1.0 or cos_q2 > 1.0:
            return None
        q2 = math.acos(max(-1.0, min(1.0, cos_q2)))
        # compute q1
        phi = math.atan2(y, x)
        cos_beta = (r_wrist**2 + l1**2 - l2**2) / (2.0 * l1 * r_wrist) if r_wrist != 0.0 else 1.0
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta) if r_wrist != 0.0 else 0.0
        q1 = phi - beta
        q3 = 0.0
        return [self.norm(q1), self.norm(q2), self.norm(q3)]

    def generate_trajectory(self, q0, qf, total_time, N):
        t = np.linspace(0.0, total_time, max(2, N))
        q0 = np.array(q0); qf = np.array(qf)
        q_traj = [list(q0 + (qf - q0) * (tt / total_time)) for tt in t]
        return t, q_traj

    def fk(self, q):
        q1, q2, q3 = q
        a1, a2, a3 = self.l1, self.l2, self.l3
        px_local = a1 * math.cos(q2) + a2 * math.cos(q2 + q3)
        py_local = a1 * math.sin(q2) + a2 * math.sin(q2 + q3)
        x = px_local * math.cos(q1) - py_local * math.sin(q1)
        y = px_local * math.sin(q1) + py_local * math.cos(q1)
        return (x, y)

    def plot_all(self, ee_path, q_traj, torques):
        try:
            # workspace
            fig1 = plt.figure()
            xs = [p[0] for p in ee_path]; ys = [p[1] for p in ee_path]
            plt.plot(xs, ys, 'o-'); plt.scatter(xs[-1], ys[-1], c='r')
            plt.title(f"{self.ns} workspace"); plt.xlabel('X'); plt.ylabel('Y'); plt.grid(True)

            # joint space
            fig2 = plt.figure()
            q_arr = np.array(q_traj); t = np.linspace(0, self.traj_time, q_arr.shape[0])
            plt.plot(t, q_arr[:,0], label='q1'); plt.plot(t, q_arr[:,1], label='q2'); plt.plot(t, q_arr[:,2], label='q3')
            plt.title(f"{self.ns} joints"); plt.legend(); plt.grid(True)

            # torques
            fig3 = plt.figure()
            torque_arr = np.array(torques)
            plt.plot(t, torque_arr[:,0], label='tau1'); plt.plot(t, torque_arr[:,1], label='tau2'); plt.plot(t, torque_arr[:,2], label='tau3')
            plt.title(f"{self.ns} torques"); plt.legend(); plt.grid(True)

            plt.show(); plt.pause(0.001)
        except Exception as e:
            self.get_logger().error(f"Plot error: {e}")

    def execute_trajectory(self, t_samples, q_traj):
        with self.exec_lock:
            js = JointState()
            js.name = ['joint1', 'joint2', 'joint3']
            rate_hz = self.exec_rate
            period = 1.0 / rate_hz
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
    node = PlanarClickToMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
