"""Launch del simulador con el mapa SaoPaulo CON obstaculos (Parte 2).

El launch del profesor (gym_bridge_launch.py) lee siempre su config/sim.yaml,
cuyo map_path apunta al mapa limpio de la Parte 1. Para correr la Parte 2 sin
modificar su repo, este launch replica sus mismos nodos (bridge, map_server,
lifecycle, RViz, robot_state_publisher) pero sobreescribe map_path hacia el
mapa con obstaculos instalado en el share de MI paquete (ftg_rv/maps/).

El resto de parametros (topicos, num_agent, poses iniciales, etc.) se sigue
leyendo del sim.yaml del profesor: unica fuente de verdad para todo lo que no
es el mapa. En launch_ros, cuando un nodo recibe varias fuentes de parametros,
la ultima gana, asi que [sim.yaml, {'map_path': ...}] aplica solo ese override.

Uso:
    ros2 launch ftg_rv sim_obs.launch.py
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    gym_share = get_package_share_directory('f1tenth_gym_ros')
    ftg_share = get_package_share_directory('ftg_rv')

    sim_config = os.path.join(gym_share, 'config', 'sim.yaml')
    # Ruta SIN extension: el bridge le agrega '.yaml' (gym) y map_img_ext (png).
    obs_map_path = os.path.join(ftg_share, 'maps', 'SaoPaulo_obs_map')

    config_dict = yaml.safe_load(open(sim_config, 'r'))
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[sim_config, {'map_path': obs_map_path}],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(gym_share, 'launch', 'gym_bridge.rviz')],
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': obs_map_path + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}],
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}],
    )
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', os.path.join(gym_share, 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')],
    )
    opp_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', os.path.join(gym_share, 'launch', 'opp_racecar.xacro')])}],
        remappings=[('/robot_description', 'opp_robot_description')],
    )

    ld = LaunchDescription()
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    if has_opp:
        ld.add_action(opp_robot_publisher)

    return ld
