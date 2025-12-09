# src/click_to_motion/click_to_motion/click_to_motion.py
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from click_to_motion_msgs.msg import WorkspacePosition

class ClickToMotion(Node):
    """
    Nodo 'maestro' simplificado: escucha clicks en /{ns}/clicked_point
    y publica un WorkspacePosition en /{ns}/workspace_target.

    Deja que los nodos ik_solver, trajectory_generator, torque_computer,
    plotter y motion_executor trabajen por tópicos (desacoplado).
    """
    def __init__(self):
        super().__init__('click_to_motion')

        # Namespace efectivo (si el launch pasa __ns, lo toma; si no, usa parámetro)
        # Preferimos usar la namespace real donde se ejecuta el nodo:
        ns = self.get_namespace().strip('/')
        if not ns:
            # fallback a parámetro si no hay namespace
            self.declare_parameter('robot_ns', '')
            ns = self.get_parameter('robot_ns').get_parameter_value().string_value.strip('/')
        self.robot_ns = ns

        self.get_logger().info(f"Click-to-Motion iniciado para namespace: '{self.robot_ns or '(root)'}'")

        # tópicos basados en namespace
        sub_topic = f"/{self.robot_ns}/clicked_point" if self.robot_ns else "/clicked_point"
        pub_topic = f"/{self.robot_ns}/workspace_target" if self.robot_ns else "/workspace_target"

        # Suscribir a clicks (PublishPoint de RViz)
        self.subscription = self.create_subscription(
            PointStamped,
            sub_topic,
            self.handle_click,
            10
        )

        # Publicador del workspace target (lo consumirá ik_solver)
        self.publisher_ = self.create_publisher(WorkspacePosition, pub_topic, 10)

        self.get_logger().info(f"Suscrito a: {sub_topic}   -> publicando en: {pub_topic}")

    def handle_click(self, msg: PointStamped):
        # Construir y publicar WorkspacePosition
        wp = WorkspacePosition()
        wp.x = msg.point.x
        wp.y = msg.point.y
        wp.z = msg.point.z
        self.publisher_.publish(wp)

        self.get_logger().info(
            f"[{self.robot_ns or 'root'}] Punto recibido: "
            f"{wp.x:.3f}, {wp.y:.3f}, {wp.z:.3f} -> publicado en workspace_target"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ClickToMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

