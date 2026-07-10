"""Nodo de vueltas y cronometro para el simulador F1Tenth.

Se suscribe solo a la odometria (/ego_racecar/odom) y no toca el LiDAR ni
/drive: es independiente del controlador Follow the Gap (reactive_node).
Se separo de reactive_node para que el oponente (una segunda instancia de
reactive_node remapeada) no imprima su propio conteo de vueltas en la
consola del video de evidencia.
"""
import rclpy
from rclpy.node import Node

import numpy as np
from nav_msgs.msg import Odometry


class LapTimerNode(Node):
    """Cuenta vueltas y cronometra el recorrido a partir de la odometria."""

    def __init__(self):
        super().__init__('lap_timer_node')

        # ---------- Parametros (ver config/params*.yaml) ----------
        self.declare_parameter('finish_line_tolerance', 4.0)
        self.declare_parameter('min_lap_time', 10.0)
        self.declare_parameter('total_laps', 10)

        p = self.get_parameter
        self.finish_line_tolerance = p('finish_line_tolerance').value
        self.min_lap_time = p('min_lap_time').value
        self.total_laps = p('total_laps').value

        # ---------- Estado de vueltas y cronometro ----------
        self.finish_line = None      # (x, y) de la meta; primer mensaje de odom
        self.lap_count = 0
        self.lap_start_time = None   # rclpy.time.Time del inicio de la vuelta
        self.total_start_time = None # rclpy.time.Time del inicio de la corrida
        self.best_lap_time = None
        # Histeresis: hay que ALEJARSE de la meta antes de poder contar el
        # siguiente cruce; evita contar varias veces el mismo paso por meta.
        self.lap_armed = False

        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        self.get_logger().info('Nodo de vueltas y cronometro iniciado.')

    def odom_callback(self, msg: Odometry):
        """Cuenta vueltas y cronometra usando la posicion del auto.

        La meta es la posicion del PRIMER mensaje de odometria (donde aparece
        el auto). Una vuelta se cuenta cuando el auto vuelve a entrar al
        circulo de radio finish_line_tolerance alrededor de la meta, con dos
        protecciones contra dobles conteos:
          - Histeresis: primero debe salir del circulo (lap_armed).
          - Tiempo minimo de vuelta (min_lap_time).
        Al completar total_laps, se imprime la mejor vuelta y se apaga este
        nodo (rclpy.shutdown()); el launch detecta la salida del proceso y
        cierra el resto (simulador, reactive_node) con el mismo evento.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
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
            if self.best_lap_time is None or lap_time < self.best_lap_time:
                self.best_lap_time = lap_time
            self.get_logger().info(
                '\n' + '=' * 46 + '\n'
                f'  VUELTA {self.lap_count} COMPLETADA\n'
                f'  Tiempo de vuelta:   {lap_time:7.2f} s\n'
                f'  Tiempo acumulado:   {total_time:7.2f} s\n'
                + '=' * 46)

            if self.lap_count >= self.total_laps:
                self.get_logger().info(
                    '\n' + '#' * 46 + '\n'
                    f'  {self.total_laps} VUELTAS COMPLETADAS - FIN DE LA PRUEBA\n'
                    f'  Mejor vuelta:       {self.best_lap_time:7.2f} s\n'
                    f'  Tiempo total:       {total_time:7.2f} s\n'
                    + '#' * 46)
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LapTimerNode()
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
