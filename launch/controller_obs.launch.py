"""Launch de la PARTE 2: simulador con obstaculos + controlador.

Levanta sim_obs.launch.py (simulador con el mapa SaoPaulo con obstaculos) y
reactive_node con el tuning conservador de config/params_obs.yaml. Es el
equivalente "todo en uno" de la Parte 2; la Parte 1 se sigue lanzando con el
simulador del profesor + controller.launch.py (params.yaml, mapa limpio).

Uso:
    ros2 launch ftg_rv controller_obs.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ftg_share = get_package_share_directory('ftg_rv')

    sim_obs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ftg_share, 'launch', 'sim_obs.launch.py')))

    reactive_node = Node(
        package='ftg_rv',
        executable='reactive_node',
        name='reactive_node',
        parameters=[os.path.join(ftg_share, 'config', 'params_obs.yaml')],
        output='screen',        # logs visibles en la consola (video de evidencia)
        emulate_tty=True,       # conserva colores/formato del logger de ROS 2
    )

    return LaunchDescription([sim_obs, reactive_node])
