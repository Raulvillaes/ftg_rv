"""Launch de la PARTE 1: simulador con el mapa limpio + controlador.

Levanta sim.launch.py (simulador con el mapa SaoPaulo limpio instalado en
este paquete) y reactive_node con el tuning de config/params.yaml, cargado
como archivo de parametros para poder tunear sin tocar el codigo.

Uso:
    ros2 launch ftg_rv controller.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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

    return LaunchDescription([sim, reactive_node])
