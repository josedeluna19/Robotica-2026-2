#!/usr/bin/env python3
"""
click_to_motion_node.py

Option A: this node only publishes JointState messages. robot_state_publisher
must be running (we assume the URDF + frame_prefix loaded by launch).

Behavior:
 - subscribes to /<ns>/clicked_point (PointStamped)
 - computes IK (planar or simple spatial)
 - generates a joint-space trajectory (linear interpolation)
 - plots: workspace path, joint trajectories, simple torque estimates
 - publishes JointState (names: ['joint1','joint2','joint3']) to /<ns>/joint_states
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

class ClickToMotionNode(Node):
    def __init__(self):
        super().__init__('click_to_motion_node')

        # Parameters (can be overridden via ros2 run --ros-args -p ...)
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('robot_type', 'planar')  # 'planar' or 'spatial'
        self.declare_parameter('l1', 0.4)
        self.declare_parameter('l2', 0.35)
        self.declare_parameter('l3', 0.35)
        self.declare_parameter('exec_rate', 50.0)  # publish rate during execution (Hz)
        self.declare_parameter('traj_time', 2.0)   # duration of trajectory (s)

        # Determine namespace (prefer the node's ROS namespace)
        param_ns = self.get_parameter('robot_namespace').get_parameter_value().string_value
        ns = self.get_namespace().strip('/')
        if ns:
            self.ns = ns
        elif param_ns:
            self.ns = param_ns
        else:
            self.ns = ''  # root

        # Read parameters
        self.robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        self.l1 = float(self.get_parameter('l1').get_parameter_value().double_value)
        self.l2 = float(self.get_parameter('l2').get_parameter_value().double_value)
        self.l3 = float(self.get_parameter('l3').get_parameter_value().double_value)
        self.exec_rate = float(self.get_parameter('exec_rate').get_parameter_value().double_value)
        self.traj_time = float(self.get_parameter('traj_time').get_parameter_value().double_value)

        # Topics (namespaced)
        click_topic = f"/{self.ns}/clicked_point" if self.ns else "/clicked_point"
        joint_states_topic = f"/{self.ns}/joint_states" if self.ns else "/joint_states"

        self.get_logger().info(f"[{self.ns}] ClickToMotion init: type={self.robot_type} l1={self.l1} l2={self.l2} l3={self.l3}")
        self.get_logger().info(f"[{self.ns}] subscribing clicks: {click_topic}  publishing joint_states: {joint_states_topic}")

        # Subscriptions & publishers
        self.click_sub = self.create_subscription(PointStamped, click_topic, self.clicked_callback, 10)
        self.joint_pub = self.create_publisher(JointState, joint_states_topic, 10)

        # Keep latest joint state (to start trajectories from current position)
        self.latest_joint = [0.0, 0.0, 0.0]
        self.have_joint = False
        self.create_subscription(JointState, joint_states_topic, self.joint_state_cb, 10)

        # Execution lock / thread
        self.exec_lock = threading.Lock()
        self.exec_thread = None

    # -----------------------
    # Callbacks
    # -----------------------
    def joint_state_cb(self, msg: JointState):
        # Only accept if contains at least three positions
        if len(msg.position) >= 3:
            self.latest_joint = list(msg.position[:3])
            self.have_joint = True

    def clicked_callback(self, msg: PointStamped):
        x = float(msg.point.x); y = float(msg.point.y); z = float(msg.point.z)
        self.get_logger().info(f"[{self.ns}] Click received: ({x:.3f}, {y:.3f}, {z:.3f})")

        # Solve IK according to robot type
        if self.robot_type == 'planar':
            q_goal = self.ik_planar(x, y)
        else:
            q_goal = self.ik_spatial(x, y, z)

        if q_goal is None:
            self.get_logger().warning(f"[{self.ns}] IK failed - point unreachable")
            return

        self.get_logger().info(f"[{self.ns}] IK goal: {q_goal}")

        q_start = self.latest_joint.copy() if self.have_joint else [0.0, 0.0, 0.0]
        t_samples, q_traj = self.generate_trajectory(q_start, q_goal, self.traj_time, int(self.exec_rate * self.traj_time))

        # Compute EE path for plotting
        ee_path = [self.fk_position(q) for q in q_traj]

        # Simple torque estimate placeholders
        torques = [[0.5 * qi for qi in q] for q in q_traj]

        # Launch plotting in background
        plot_thread = threading.Thread(target=self.plot_all, args=(ee_path, q_traj, torques))
        plot_thread.daemon = True
        plot_thread.start()

        # Execute trajectory (publish JointState)
        if self.exec_lock.locked():
            self.get_logger().warning(f"[{self.ns}] Another execution already running. Skipping this click.")
            return

        self.exec_thread = threading.Thread(target=self.execute_trajectory, args=(t_samples, q_traj))
        self.exec_thread.daemon = True
        self.exec_thread.start()

    # -----------------------
    # IK solvers
    # -----------------------
    def ik_planar(self, x, y):
        """
        Simple 3-DOF planar IK:
        - q1 rotates base (yaw around z)
        - q2,q3 are planar elbow joints (we solve for q2 using cos law)
        - q3 set to 0 (end effector orientation not controlled)
        """
        r = math.hypot(x, y)
        # treat l3 as EE offset along last link: aim wrist at r-l3
        r_wrist = max(0.0, r - self.l3)
        l1, l2 = self.l1, self.l2
        # reachability
        cos_q2 = (r_wrist**2 - l1**2 - l2**2) / (2.0 * l1 * l2)
        if cos_q2 < -1.0 or cos_q2 > 1.0:
            return None
        q2 = math.acos(max(-1.0, min(1.0, cos_q2)))
        # compute q1 using geometry
        k1 = l1 + l2 * math.cos(q2)
        k2 = l2 * math.sin(q2)
        phi = math.atan2(y, x)
        alpha = math.atan2(k2, k1) if (k1 != 0.0 or k2 != 0.0) else 0.0
        # compute distance-based beta for wrist (avoid division by zero)
        if r_wrist != 0.0:
            cos_beta = (r_wrist**2 + l1**2 - l2**2) / (2.0 * l1 * r_wrist)
            cos_beta = max(-1.0, min(1.0, cos_beta))
            beta = math.acos(cos_beta)
        else:
            beta = 0.0
        q1 = phi - beta
        # choose elbow-down solution (you can flip sign of q2 if you prefer elbow-up)
        q3 = 0.0
        return [self.normalize_angle(q1), self.normalize_angle(q2), self.normalize_angle(q3)]

    def ik_spatial(self, x, y, z):
        """
        Simple numeric spatial IK: q1 = atan2(y,x) (base yaw),
        then solve a 2-DOF planar arm for (r,z) with links l2,l3 and an offset l1 (vertical).
        Uses a coarse search.
        """
        q1 = math.atan2(y, x)
        r = math.hypot(x, y)
        tx = r
        tz = z
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
        q2 = best[2]; q3 = best[3]
        return [self.normalize_angle(q1), self.normalize_angle(q2), self.normalize_angle(q3)]

    # -----------------------
    # Trajectory generation & FK
    # -----------------------
    def generate_trajectory(self, q0, qf, total_time, N):
        t = np.linspace(0.0, total_time, max(2, N))
        q0 = np.array(q0); qf = np.array(qf)
        q_traj = [list(q0 + (qf - q0) * (tt / total_time)) for tt in t]
        return t, q_traj

    def fk_position(self, q):
        # For plotting only. For planar robots returns (x,y). For spatial returns (x,y,z).
        q1, q2, q3 = q
        if self.robot_type == 'planar':
            # local planar chain uses q2,q3, q1 rotates whole plane
            a1, a2, a3 = self.l1, self.l2, self.l3
            px_local = a1 * math.cos(q2) + a2 * math.cos(q2 + q3)
            py_local = a1 * math.sin(q2) + a2 * math.sin(q2 + q3)
            x = px_local * math.cos(q1) - py_local * math.sin(q1)
            y = px_local * math.sin(q1) + py_local * math.cos(q1)
            return (x, y)
        else:
            a1, a2, a3 = self.l1, self.l2, self.l3
            planar_x = a2 * math.cos(q2) + a3 * math.cos(q2 + q3)
            planar_z = a1 + a2 * math.sin(q2) + a3 * math.sin(q2 + q3)
            x = planar_x * math.cos(q1); y = planar_x * math.sin(q1); z = planar_z
            return (x, y, z)

    # -----------------------
    # Plotting
    # -----------------------
    def plot_all(self, ee_path, q_traj, torques):
        try:
            # Workspace
            fig1 = plt.figure(figsize=(5,4))
            if self.robot_type == 'planar':
                xs = [p[0] for p in ee_path]
                ys = [p[1] for p in ee_path]
                plt.plot(xs, ys, 'o-', label='EE path')
                plt.scatter([xs[-1]], [ys[-1]], c='r', label='goal')
                plt.title(f"{self.ns} Workspace (XY)")
                plt.xlabel('X (m)'); plt.ylabel('Y (m)')
                plt.grid(True); plt.legend()
            else:
                xs = [p[0] for p in ee_path]
                zs = [p[2] for p in ee_path]
                plt.plot(xs, zs, 'o-', label='EE x-z')
                plt.title(f"{self.ns} Workspace (X-Z)")
                plt.xlabel('X (m)'); plt.ylabel('Z (m)'); plt.grid(True)

            # Joint space
            fig2 = plt.figure(figsize=(6,4))
            q_arr = np.array(q_traj)
            t = np.linspace(0, self.traj_time, q_arr.shape[0])
            plt.plot(t, q_arr[:,0], label='q1'); plt.plot(t, q_arr[:,1], label='q2'); plt.plot(t, q_arr[:,2], label='q3')
            plt.title(f"{self.ns} Joint trajectories"); plt.xlabel('Time (s)'); plt.ylabel('Radians'); plt.legend(); plt.grid(True)

            # Torques placeholder
            fig3 = plt.figure(figsize=(6,3))
            torque_arr = np.array(torques)
            plt.plot(t, torque_arr[:,0], label='tau1'); plt.plot(t, torque_arr[:,1], label='tau2'); plt.plot(t, torque_arr[:,2], label='tau3')
            plt.title(f"{self.ns} Estimated torques (placeholder)"); plt.xlabel('Time (s)'); plt.ylabel('Torque (arb.)'); plt.legend(); plt.grid(True)

            plt.show(); plt.pause(0.001)
        except Exception as e:
            self.get_logger().error(f"[{self.ns}] Plotting failed: {e}")

    # -----------------------
    # Execution (publishing JointState)
    # -----------------------
    def execute_trajectory(self, t_samples, q_traj):
        with self.exec_lock:
            rate_hz = self.exec_rate
            period = 1.0 / rate_hz
            js = JointState()
            js.name = ['joint1', 'joint2', 'joint3']  # MUST match URDF joint names
            start_time = time.time()
            for q in q_traj:
                js.header.stamp = self.get_clock().now().to_msg()
                js.position = list(q)
                try:
                    self.joint_pub.publish(js)
                except Exception as e:
                    self.get_logger().error(f"[{self.ns}] publish failed: {e}")
                # maintain execution rate (simple sleep)
                time.sleep(period)
            self.get_logger().info(f"[{self.ns}] Trajectory execution finished.")

    # -----------------------
    # Helpers
    # -----------------------
    def normalize_angle(self, a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    node = ClickToMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
