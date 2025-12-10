#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import JointState

from .kinematics import Robot2Kinematics
from .dynamics import Robot2Dynamics


class ManipulatorController(Node):
    """
    Controlador para el Robot 2:
      - Base rotatoria en Z (turret_joint_r2)
      - Manipulador RR vertical (shoulder_joint_r2, arm_joint_r2)
    Usa:
      - RobotKinematics (espacio de trabajo: [x, y, z])
      - RobotDynamics  (pares articulares via Lagrange)
    """

    def __init__(self):
        super().__init__("manipulator_controller")

        # ---- CONFIG: ¿quieres ver gráficas desde el nodo? ----
        # Pon esto en True si quieres que se abran las gráficas,
        # en False para que NO se abra ninguna.
        self.enable_plots = True

        # --- CINEMÁTICA ---
        self.kinematics = Robot2Kinematics()
        try:
            self.kinematics.redirect_print(self.get_logger().info)
        except AttributeError:
            pass

        self.kinematics.direct_kinematics()

        # --- DINÁMICA ---
        self.dynamics = Robot2Dynamics()
        try:
            self.dynamics.redirect_print(self.get_logger().info)
        except AttributeError:
            pass

        try:
            self.dynamics.define_kinematics(self.kinematics)
        except AttributeError:
            pass

        self.dynamics.define_dynamics()

        # Estado
        self.moving = False
        self.count=0

        # Suscripciones
        self.end_effector_goal_subscriber = self.create_subscription(
            Twist, "/end_effector_goal", self.end_effector_callback, 10
        )
        self.clicked_point_subscriber = self.create_subscription(
            PointStamped, "/clicked_point", self.clicked_point_callback, 10
        )
        self.joint_states_subscriber = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        # Publicador de objetivos de juntas
        self.joint_goals_publisher = self.create_publisher(
            JointState, "/joint_goals", 10
        )

        self.get_logger().info("ManipulatorController para Robot 2 inicializado")

    # -------------------------------------------------------------------------
    #  CALLBACKS DE ENTRADA
    # -------------------------------------------------------------------------

    def end_effector_callback(self, msg: Twist):
        """Objetivo en espacio de trabajo desde /end_effector_goal."""
        if self.moving:
            self.get_logger().warning("Trayectoria en ejecución, mensaje ignorado")
            return

        if not hasattr(self, "current_joint_states"):
            self.get_logger().warning("Aún no se recibe /joint_states, mensaje ignorado")
            return

        self.moving = True
        self.get_logger().info(
            f"Objetivo Twist recibido: x={msg.linear.x:.3f}, "
            f"y={msg.linear.y:.3f}, z={msg.linear.z:.3f}"
        )

        q_in = list(self.current_joint_states.position)
        xi_fn = [msg.linear.x, msg.linear.y, msg.linear.z]

        self.kinematics.trajectory_generator(
            q_in=q_in,
            xi_fn=xi_fn,
            duration=4.0
        )

        self.kinematics.inverse_kinematics()

        # Opcional: solo si enable_plots = True
        if self.enable_plots:
            try:
                self.kinematics.ws_graph()
                self.kinematics.q_graph()
            except Exception as e:
                self.get_logger().warning(f"No se pudieron mostrar gráficas: {e}")

        self._prepare_trajectory_timer()

    def clicked_point_callback(self, msg: PointStamped):
        """Objetivo clickeado en RViz via /clicked_point."""
        if self.moving:
            self.get_logger().warning("Trayectoria en ejecución, mensaje ignorado")
            return

        if not hasattr(self, "current_joint_states"):
            self.get_logger().warning("Aún no se recibe /joint_states, mensaje ignorado")
            return

        self.moving = True
        self.get_logger().info(
            f"Punto clickeado: x={msg.point.x:.3f}, "
            f"y={msg.point.y:.3f}, z={msg.point.z:.3f}"
        )

        q_in = list(self.current_joint_states.position)
        xi_fn = [msg.point.x, msg.point.y, msg.point.z]

        self.kinematics.trajectory_generator(
            q_in=q_in,
            xi_fn=xi_fn,
            duration=4.0
        )
        self.kinematics.inverse_kinematics()

        # Dinámica
        try:
            self.dynamics.lagrange_effort_generator()
        except Exception as e:
            self.get_logger().warning(f"Error generando dinámica de Lagrange: {e}")

        # Opcional: solo si enable_plots = True
        if self.enable_plots:
            try:
                self.kinematics.ws_graph()
                self.kinematics.q_graph()
                self.dynamics.effort_graph()
            except Exception as e:
                self.get_logger().warning(f"No se pudieron mostrar gráficas: {e}")

        self._prepare_trajectory_timer()

    def joint_states_callback(self, msg: JointState):
        """Guarda el estado actual de las juntas."""
        self.current_joint_states = msg

    # -------------------------------------------------------------------------
    #  PUBLICACIÓN DE TRAYECTORIA EN /joint_goals
    # -------------------------------------------------------------------------

    def _prepare_trajectory_timer(self):
        """Inicializa el timer para publicar q_m en /joint_goals."""
        self.count = 0
        self.joint_goals = JointState()
        self.joint_goals.name = [
            "turret_joint_r2",
            "shoulder_joint_r2",
            "arm_joint_r2"
        ]

        self.get_logger().info("Iniciando publicación de trayectoria articular")

        self.position_publisher_timer = self.create_timer(
            float(self.kinematics.dt),
            self.trajectory_publisher_callback
        )

    def trajectory_publisher_callback(self):
        """Publica muestras de la trayectoria en /joint_goals."""
        self.joint_goals.header.stamp = self.get_clock().now().to_msg()
        th1 = float(self.kinematics.q_m[0, self.count])
        th2 = float(self.kinematics.q_m[1, self.count])
        th3 = float(self.kinematics.q_m[2, self.count])
        """try:
            th1 = float(self.kinematics.q_m[0, self.count])
            th2 = float(self.kinematics.q_m[1, self.count])
            th3 = float(self.kinematics.q_m[2, self.count])
        except IndexError:
            self.get_logger().warning(
                "Índice fuera de rango al publicar trayectoria, cancelando"
            )
            self.position_publisher_timer.cancel()
            self.moving = False
            return"""

        self.joint_goals.position = [th1, th2, th3]
        self.joint_goals_publisher.publish(self.joint_goals)

        self.count += 1

        if self.count >= self.kinematics.samples:
            self.get_logger().info("Trayectoria articular finalizada")
            self.position_publisher_timer.cancel()
            self.moving = False


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()