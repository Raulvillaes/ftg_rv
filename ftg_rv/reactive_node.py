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
        self.declare_parameter('brake_distance', 4.0)
        self.declare_parameter('full_speed_distance', 7.0)
        self.declare_parameter('fov_angle', 90.0)
        self.declare_parameter('smoothing_window', 5)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('gap_threshold', 2.0)
        self.declare_parameter('bubble_radius', 0.4)
        self.declare_parameter('best_point_bias', 0.5)
        self.declare_parameter('finish_line_tolerance', 1.0)
        self.declare_parameter('min_lap_time', 10.0)

        p = self.get_parameter
        self.max_speed = p('max_speed').value
        self.mid_speed = p('mid_speed').value
        self.corner_speed = p('corner_speed').value
        self.steer_low = np.radians(p('steering_threshold_low').value)
        self.steer_high = np.radians(p('steering_threshold_high').value)
        self.brake_distance = p('brake_distance').value
        self.full_speed_distance = p('full_speed_distance').value
        self.fov_angle = np.radians(p('fov_angle').value)
        self.smoothing_window = p('smoothing_window').value
        self.max_range = p('max_range').value
        self.gap_threshold = p('gap_threshold').value
        self.bubble_radius = p('bubble_radius').value
        self.best_point_bias = p('best_point_bias').value
        self.finish_line_tolerance = p('finish_line_tolerance').value
        self.min_lap_time = p('min_lap_time').value

        # Indices del recorte de FOV; se calculan con el primer scan recibido
        # (dependen de angle_min/angle_increment, que no conocemos hasta ahi).
        self.fov_start = None
        self.fov_end = None

        # ---------- Estado de vueltas y cronometro ----------
        self.finish_line = None      # (x, y) de la meta; primer mensaje de odom
        self.lap_count = 0
        self.lap_start_time = None   # rclpy.time.Time del inicio de la vuelta
        self.total_start_time = None # rclpy.time.Time del inicio de la corrida
        # Histeresis: hay que ALEJARSE de la meta antes de poder contar el
        # siguiente cruce; evita contar varias veces el mismo paso por meta.
        self.lap_armed = False

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
        """Procesa cada scan del LiDAR con Follow the Gap y publica /drive.

        Pipeline: recorte de FOV -> limpieza y suavizado -> burbuja de
        seguridad sobre el punto mas cercano -> gap libre mas largo ->
        mejor punto del gap -> angulo de direccion + velocidad.
        """
        if self.fov_start is None:
            self._compute_fov_indices(scan)

        proc_ranges = self.preprocess_lidar(scan)

        # Espacio libre directamente al frente (media de los rayos centrales,
        # ANTES de la burbuja para que esta no lo anule). Gobierna el frenado
        # al acercarse a una curva aunque el volante siga recto.
        center = len(proc_ranges) // 2
        forward_clearance = float(np.mean(proc_ranges[center - 5:center + 6]))

        # Burbuja de seguridad: anula un radio fisico alrededor del punto
        # mas cercano para que el auto lo esquive en vez de rozarlo.
        closest_idx = int(np.argmin(proc_ranges))
        closest_dist = proc_ranges[closest_idx]
        proc_ranges = self.apply_safety_bubble(
            proc_ranges, closest_idx, closest_dist, scan.angle_increment)

        gap = self.find_max_gap(proc_ranges)
        if gap is None:
            # Sin gap libre: emergencia. Apuntar al rayo mas largo que quede
            # e ir a velocidad de curva para no empotrarse.
            best_idx = int(np.argmax(proc_ranges))
            steering_angle = self.index_to_angle(best_idx, scan)
            self.publish_drive(steering_angle, self.corner_speed)
            return

        best_idx = self.find_best_point(gap[0], gap[1], proc_ranges)
        steering_angle = self.index_to_angle(best_idx, scan)
        speed = self.compute_speed(steering_angle, forward_clearance)
        self.publish_drive(steering_angle, speed)

    # ------------------------------------------------------------------
    # Follow the Gap
    # ------------------------------------------------------------------
    def _compute_fov_indices(self, scan):
        """Calcula (una sola vez) los indices que recortan el scan a +-fov_angle.

        El LiDAR del simulador cubre ~270 grados; lo que queda detras de los
        hombros del auto no sirve para decidir hacia donde avanzar y ademas
        puede confundir al algoritmo (gaps detras del auto).
        """
        n = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1
        center = n // 2  # rayo que apunta al frente (angulo 0)
        half_fov_rays = int(self.fov_angle / scan.angle_increment)
        self.fov_start = max(0, center - half_fov_rays)
        self.fov_end = min(n - 1, center + half_fov_rays)
        self.get_logger().info(
            f'FOV recortado a rayos [{self.fov_start}, {self.fov_end}] '
            f'de {n} (+-{np.degrees(self.fov_angle):.0f} grados)')

    def preprocess_lidar(self, scan):
        """Limpia y suaviza el scan ya recortado al FOV util.

        1. Recorta a [fov_start, fov_end].
        2. NaN/inf -> 0 (lecturas invalidas se tratan como obstaculo).
        3. Satura a max_range: una lectura de 30 m no debe dominar la
           eleccion del mejor punto.
        4. Media movil para eliminar ruido puntual del sensor.
        """
        ranges = np.array(scan.ranges[self.fov_start:self.fov_end + 1],
                          dtype=np.float64)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_range,
                               neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_range)
        window = int(self.smoothing_window)
        if window > 1:
            kernel = np.ones(window) / window
            ranges = np.convolve(ranges, kernel, mode='same')
        return ranges

    def apply_safety_bubble(self, ranges, center_idx, center_dist, angle_inc):
        """Pone a cero los rayos dentro de bubble_radius metros del punto
        mas cercano.

        El radio fisico se convierte a numero de rayos con el angulo que
        subtiende la burbuja a esa distancia: cuanto mas cerca el obstaculo,
        mas rayos ocupa la misma burbuja.
        """
        if center_dist <= 0.0:
            bubble_rays = int(np.radians(5.0) / angle_inc)  # minimo razonable
        else:
            bubble_rays = int(np.arctan2(self.bubble_radius, center_dist)
                              / angle_inc)
        lo = max(0, center_idx - bubble_rays)
        hi = min(len(ranges) - 1, center_idx + bubble_rays)
        ranges[lo:hi + 1] = 0.0
        return ranges

    def find_max_gap(self, ranges):
        """Devuelve (inicio, fin) de la secuencia contigua mas larga de rayos
        "libres" (distancia > gap_threshold), o None si no hay ninguno.
        """
        free = ranges > self.gap_threshold
        if not np.any(free):
            return None
        # Bordes de las rachas de True: diff sobre el array con centinelas.
        edges = np.diff(np.concatenate(([0], free.astype(int), [0])))
        starts = np.where(edges == 1)[0]
        ends = np.where(edges == -1)[0] - 1
        longest = int(np.argmax(ends - starts))
        return int(starts[longest]), int(ends[longest])

    def find_best_point(self, start_i, end_i, ranges):
        """Elige el punto objetivo dentro del gap.

        Mezcla el punto mas lejano (rapido, apura las curvas) con el centro
        del gap (seguro, se aleja de las paredes) segun best_point_bias:
        0.0 = solo el mas lejano, 1.0 = solo el centro.

        En rectas largas muchos rayos saturan a max_range y empatan como
        "mas lejano"; argmax devolveria el primero (el borde del empate) y
        el auto zigzaguearia. Entre los empatados se elige el mas cercano
        al centro del gap para apuntar de frente.
        """
        window = ranges[start_i:end_i + 1]
        center_idx = (start_i + end_i) // 2
        # Candidatos: rayos practicamente empatados con el maximo (98%).
        candidates = np.where(window >= 0.98 * np.max(window))[0] + start_i
        furthest_idx = int(candidates[np.argmin(np.abs(candidates - center_idx))])
        bias = self.best_point_bias
        return int(round((1.0 - bias) * furthest_idx + bias * center_idx))

    def index_to_angle(self, idx, scan):
        """Convierte un indice del array recortado al angulo real del rayo."""
        return scan.angle_min + (self.fov_start + idx) * scan.angle_increment

    def compute_speed(self, steering_angle, forward_clearance):
        """Velocidad = minimo entre dos criterios independientes.

        1. Por angulo de volante: cuanto mas girado, mas despacio.
        2. Por espacio libre al frente: lineal entre corner_speed (a
           brake_distance o menos) y max_speed (a full_speed_distance o
           mas). Sin esto el auto acelera en el vertice de la curva (volante
           momentaneamente recto hacia la salida) y no frena al final de
           las rectas.
        """
        abs_steer = abs(steering_angle)
        if abs_steer < self.steer_low:
            speed_steer = self.max_speed
        elif abs_steer < self.steer_high:
            speed_steer = self.mid_speed
        else:
            speed_steer = self.corner_speed

        frac = (forward_clearance - self.brake_distance) \
            / (self.full_speed_distance - self.brake_distance)
        frac = float(np.clip(frac, 0.0, 1.0))
        speed_clearance = self.corner_speed \
            + frac * (self.max_speed - self.corner_speed)

        return min(speed_steer, speed_clearance)

    def odom_callback(self, odom):
        """Cuenta vueltas y cronometra usando la posicion del auto.

        La meta es la posicion del PRIMER mensaje de odometria (donde aparece
        el auto). Una vuelta se cuenta cuando el auto vuelve a entrar al
        circulo de radio finish_line_tolerance alrededor de la meta, con dos
        protecciones contra dobles conteos:
          - Histeresis: primero debe salir del circulo (lap_armed).
          - Tiempo minimo de vuelta (min_lap_time).
        """
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        now = self.get_clock().now()

        # Primer mensaje: fijar la meta y arrancar el cronometro.
        if self.finish_line is None:
            self.finish_line = (x, y)
            self.lap_start_time = now
            self.total_start_time = now
            self.get_logger().info(
                f'Meta fijada en ({x:.2f}, {y:.2f}). Cronometro iniciado.')
            return

        dist_to_finish = np.hypot(x - self.finish_line[0],
                                  y - self.finish_line[1])

        # Armar el conteo cuando el auto se aleja claramente de la meta.
        if not self.lap_armed:
            if dist_to_finish > 2.0 * self.finish_line_tolerance:
                self.lap_armed = True
            return

        # Cruce de meta: armado + dentro del circulo + vuelta plausible.
        lap_time = (now - self.lap_start_time).nanoseconds * 1e-9
        if dist_to_finish < self.finish_line_tolerance \
                and lap_time > self.min_lap_time:
            self.lap_count += 1
            total_time = (now - self.total_start_time).nanoseconds * 1e-9
            self.lap_start_time = now
            self.lap_armed = False
            self.get_logger().info(
                '\n' + '=' * 46 + '\n'
                f'  VUELTA {self.lap_count} COMPLETADA\n'
                f'  Tiempo de vuelta:   {lap_time:7.2f} s\n'
                f'  Tiempo acumulado:   {total_time:7.2f} s\n'
                + '=' * 46)

    # ------------------------------------------------------------------
    # Actuacion
    # ------------------------------------------------------------------
    def publish_drive(self, steering_angle, speed):
        """Publica un AckermannDriveStamped con el angulo y velocidad dados."""
        # El servo de direccion del F1Tenth satura en ~0.42 rad (24 grados).
        steering_angle = float(np.clip(steering_angle, -0.42, 0.42))
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
        # El launch puede haber cerrado ya el contexto (Ctrl+C / SIGTERM);
        # llamar a shutdown() dos veces lanza RCLError.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
