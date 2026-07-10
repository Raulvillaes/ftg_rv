"""Launch de la PARTE 2 con OPONENTE: obstaculos fijos + vehiculo movil.

Levanta el simulador con el mapa con obstaculos y num_agent=2, y DOS
instancias del mismo controlador Follow the Gap:

- Ego: reactive_node con el tuning de config/params_obs.yaml, en los topicos
  de siempre (/scan, /ego_racecar/odom, /drive).
- Oponente: el MISMO ejecutable, remapeado a los topicos del segundo coche
  (/opp_scan, /opp_racecar/odom, /opp_drive) y con el tuning mas lento de
  config/params_opp.yaml. No hay codigo nuevo: solo remap + parametros.

El oponente es obligatorio con num_agent=2: el bridge no da un paso de
simulacion hasta recibir comandos en /drive Y en /opp_drive, asi que sin su
controlador se congela todo, incluido el ego.

Uso:
    ros2 launch ftg_rv controller_opp.launch.py
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
            os.path.join(ftg_share, 'launch', 'sim_obs.launch.py')),
        launch_arguments={'num_agent': '2'}.items(),
    )

    ego_node = Node(
        package='ftg_rv',
        executable='reactive_node',
        name='reactive_node',
        parameters=[os.path.join(ftg_share, 'config', 'params_obs.yaml')],
        output='screen',        # logs visibles en la consola (video de evidencia)
        emulate_tty=True,       # conserva colores/formato del logger de ROS 2
    )

    opp_node = Node(
        package='ftg_rv',
        executable='reactive_node',
        name='opp_reactive_node',
        parameters=[os.path.join(ftg_share, 'config', 'params_opp.yaml')],
        remappings=[
            ('/scan', '/opp_scan'),
            ('/ego_racecar/odom', '/opp_racecar/odom'),
            ('/drive', '/opp_drive'),
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([sim_obs, ego_node, opp_node])
