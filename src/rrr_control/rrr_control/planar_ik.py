import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
import math
import matplotlib.pyplot as plt
import time

# Longitudes del robot planar
L1 = 0.30
L2 = 0.30
L3 = 0.30

class PlanarIK(Node):

    def __init__(self):
        super().__init__('planar_ik')

        # Suscribirse al punto clickeado
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_callback, 10)

        # Publicador de joint states
        self.pub_joints = self.create_publisher(JointState, 'joint_states', 10)

        # Datos para gráficas
        self.traj_x = []
        self.traj_y = []
        self.traj_q = []
        self.start_time = time.time()

        self.get_logger().info("Nodo IK Planar iniciado")

    def clicked_callback(self, msg):

        x = msg.point.x
        y = msg.point.y

        self.get_logger().info(f"IK objetivo recibido: x={x}, y={y}")

        # IK por fórmula cerrada
        solution = self.solve_ik(x, y)
        if solution is None:
            self.get_logger().warn("Punto fuera del alcance")
            return

        q1, q2, q3 = solution

        # Publicar joint states
        js = JointState()
        js.name = ["joint1", "joint2", "joint3"]
        js.position = [q1, q2, q3]
        js.header.stamp = self.get_clock().now().to_msg()
        self.pub_joints.publish(js)

        # Guardar datos
        self.traj_x.append(x)
        self.traj_y.append(y)
        self.traj_q.append([q1, q2, q3])

        self.plot_data()

    def solve_ik(self, x, y):
        """IK para un robot 3R planar"""

        # Distancia al objetivo
        r = math.sqrt(x**2 + y**2)
        if r > (L1 + L2 + L3):
            return None

        # Simplificación: L23 = L2 + L3
        L23 = L2 + L3

        # Ley del coseno
        cos_q2 = (r*r - L1*L1 - L23*L23) / (2 * L1 * L23)
        if abs(cos_q2) > 1:
            return None

        q2 = math.acos(cos_q2)

        # Ángulo inicial
        phi = math.atan2(y, x)
        k1 = L1 + L23*math.cos(q2)
        k2 = L23 * math.sin(q2)

        q1 = phi - math.atan2(k2, k1)

        # Distribuir q2 entre joint2 y joint3
        q2_real = q2 * (L2 / (L2 + L3))
        q3_real = q2 - q2_real

        return q1, q2_real, q3_real

    def plot_data(self):

        plt.figure(figsize=(10,5))

        # Espacio de trabajo
        plt.subplot(1,2,1)
        plt.plot(self.traj_x, self.traj_y, 'o-')
        plt.title("Trayectoria en el espacio de trabajo")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)

        # Espacio articular
        plt.subplot(1,2,2)
        q1 = [q[0] for q in self.traj_q]
        q2 = [q[1] for q in self.traj_q]
        q3 = [q[2] for q in self.traj_q]
        plt.plot(q1, label="q1")
        plt.plot(q2, label="q2")
        plt.plot(q3, label="q3")
        plt.title("Espacio articular")
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PlanarIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

