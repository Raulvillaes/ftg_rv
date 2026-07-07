"""Nodo reactivo Follow the Gap para el simulador F1Tenth.

Se suscribe al LiDAR (/scan) y a la odometria (/ego_racecar/odom) y publica
comandos de conduccion (/drive). Implementa:
  1. Algoritmo Follow the Gap (percepcion + control).
  2. Contador de vueltas por cruce de meta con histeresis.
  3. Cronometro por vuelta y tiempo acumulado.
"""
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class ReactiveNode(Node):
    """Controlador reactivo Follow the Gap con conteo y cronometraje de vueltas."""

    def __init__(self):
        super().__init__('reactive_node')

        # ---------- Parametros (ver config/params.yaml) ----------
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('mid_speed', 3.0)
        self.declare_parameter('corner_speed', 1.5)
        self.declare_parameter('steering_threshold_low', 10.0)
        self.declare_parameter('steering_threshold_high', 20.0)
        self.declare_parameter('fov_angle', 90.0)
        self.declare_parameter('smoothing_window', 5)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('gap_threshold', 2.0)
        self.declare_parameter('bubble_radius', 0.4)
        self.declare_parameter('finish_line_tolerance', 1.0)
        self.declare_parameter('min_lap_time', 10.0)

        p = self.get_parameter
        self.max_speed = p('max_speed').value
        self.mid_speed = p('mid_speed').value
        self.corner_speed = p('corner_speed').value
        self.steer_low = np.radians(p('steering_threshold_low').value)
        self.steer_high = np.radians(p('steering_threshold_high').value)
        self.fov_angle = np.radians(p('fov_angle').value)
        self.smoothing_window = p('smoothing_window').value
        self.max_range = p('max_range').value
        self.gap_threshold = p('gap_threshold').value
        self.bubble_radius = p('bubble_radius').value
        self.finish_line_tolerance = p('finish_line_tolerance').value
        self.min_lap_time = p('min_lap_time').value

        # ---------- Comunicacion ----------
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('Nodo Follow the Gap iniciado. '
                               f'max_speed={self.max_speed} m/s')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def lidar_callback(self, scan):
        """Procesa cada scan del LiDAR y publica el comando de conduccion."""
        # Version minima (Bloque 1): avanzar recto y lento para validar la
        # tuberia scan -> nodo -> drive. El algoritmo llega en el Bloque 2.
        self.publish_drive(steering_angle=0.0, speed=1.0)

    def odom_callback(self, odom):
        """Recibe la odometria; aqui viviran las vueltas y el cronometro."""
        pass

    # ------------------------------------------------------------------
    # Actuacion
    # ------------------------------------------------------------------
    def publish_drive(self, steering_angle, speed):
        """Publica un AckermannDriveStamped con el angulo y velocidad dados."""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = float(steering_angle)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
