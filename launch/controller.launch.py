"""Launch de la PARTE 1: simulador con el mapa limpio + controlador.

Levanta sim.launch.py (simulador con el mapa SaoPaulo limpio instalado en
este paquete), reactive_node (Follow the Gap) y lap_timer_node (contador de
vueltas y cronometro), ambos con el tuning de config/params.yaml, cargado
como archivo de parametros para poder tunear sin tocar el codigo.

Uso:
    ros2 launch ftg_rv controller.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ftg_share = get_package_share_directory('ftg_rv')

    # Simulador con el mapa limpio (los defaults de sim.launch.py:
    # map:=SaoPaulo_map, stheta:=0.0, num_agent:=1).
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ftg_share, 'launch', 'sim.launch.py')))

    reactive_node = Node(
        package='ftg_rv',
        executable='reactive_node',
        name='reactive_node',
        parameters=[os.path.join(ftg_share, 'config', 'params.yaml')],
        output='screen',        # logs visibles en la consola (video de evidencia)
        emulate_tty=True,       # conserva colores/formato del logger de ROS 2
    )

    lap_timer_node = Node(
        package='ftg_rv',
        executable='lap_timer_node',
        name='lap_timer_node',
        parameters=[os.path.join(ftg_share, 'config', 'params.yaml')],
        output='screen',        # logs visibles en la consola (video de evidencia)
        emulate_tty=True,
    )

    # Al completar total_laps, lap_timer_node imprime la mejor vuelta y se
    # cierra solo (rclpy.shutdown()); este manejador detecta esa salida y
    # apaga el resto (simulador, reactive_node) con el mismo evento.
    shutdown_on_laps_done = RegisterEventHandler(OnProcessExit(
        target_action=lap_timer_node,
        on_exit=[EmitEvent(event=Shutdown(
            reason='10 vueltas completadas'))],
    ))

    return LaunchDescription(
        [sim, reactive_node, lap_timer_node, shutdown_on_laps_done])
