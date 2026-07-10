"""Launch de la PARTE 2: simulador con obstaculos + controlador.

Levanta sim.launch.py con el mapa SaoPaulo con obstaculos (y el spawn girado
-60 grados para no arrancar de frente al primer obstaculo) y reactive_node
con el tuning conservador de config/params_obs.yaml. Es el equivalente de la
Parte 2 al todo-en-uno de controller.launch.py (params.yaml, mapa limpio).

Uso:
    ros2 launch ftg_rv controller_obs.launch.py
"""
import math
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
            os.path.join(ftg_share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'map': 'SaoPaulo_obs_map',
            'stheta': str(math.radians(-60.0)),
        }.items(),
    )

    reactive_node = Node(
        package='ftg_rv',
        executable='reactive_node',
        name='reactive_node',
        parameters=[os.path.join(ftg_share, 'config', 'params_obs.yaml')],
        output='screen',        # logs visibles en la consola (video de evidencia)
        emulate_tty=True,       # conserva colores/formato del logger de ROS 2
    )

    return LaunchDescription([sim_obs, reactive_node])
