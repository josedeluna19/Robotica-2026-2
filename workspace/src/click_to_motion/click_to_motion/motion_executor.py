# src/click_to_motion/click_to_motion/motion_executor.py
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class MotionExecutor(Node):

    def __init__(self):
        super().__init__('motion_executor')
        ns = self.get_namespace().strip('/')
        if ns == "":
            ns = "robot"

        pub_topic = f"/{ns}/joint_trajectory_controller/joint_trajectory"
        self.get_logger().info(f"MotionExecutor publishing to: {pub_topic}")
        self.publisher_ = self.create_publisher(JointTrajectory, pub_topic, 10)

        sub_topic = f"/{ns}/trajectory_out"
        self.subscription = self.create_subscription(
            JointTrajectory,
            sub_topic,
            self.trajectory_callback,
            10
        )

        self.get_logger().info(f"Motion executor listo para '{ns}', suscrito a: {sub_topic}")

    def trajectory_callback(self, msg: JointTrajectory):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = msg.joint_names
        traj_msg.points = msg.points  # forward directly
        self.publisher_.publish(traj_msg)
        self.get_logger().info("→ Trayectoria publicada al controlador.")

def main(args=None):
    rclpy.init(args=args)
    node = MotionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
