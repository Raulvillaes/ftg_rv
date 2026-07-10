"""Launch del simulador con el mapa SaoPaulo CON obstaculos (Parte 2).

El launch del profesor (gym_bridge_launch.py) lee siempre su config/sim.yaml,
cuyo map_path apunta al mapa limpio de la Parte 1. Para correr la Parte 2 sin
modificar su repo, este launch replica sus mismos nodos (bridge, map_server,
lifecycle, RViz, robot_state_publisher) pero sobreescribe los parametros
propios de este escenario: map_path (el mapa con obstaculos instalado en el
share de MI paquete, ftg_rv/maps/) y stheta (orientacion inicial del ego,
girada para no arrancar de frente al primer obstaculo).

El resto de parametros (topicos, etc.) se sigue leyendo del sim.yaml del
profesor: unica fuente de verdad para todo lo que no depende del escenario.
En launch_ros, cuando un nodo recibe varias fuentes de parametros, la ultima
gana, asi que [sim.yaml, overrides] aplica solo esos overrides.

Acepta el argumento num_agent ('1' por defecto): con '2' el bridge simula
ademas el coche oponente (y se publica su robot_description). OJO: con
num_agent=2 el simulador no avanza hasta que ALGUIEN publique en /opp_drive,
asi que este launch con '2' se usa desde controller_opp.launch.py, que lanza
los dos controladores.

Uso:
    ros2 launch ftg_rv sim_obs.launch.py                # solo ego
    ros2 launch ftg_rv sim_obs.launch.py num_agent:=2   # ego + oponente
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gym_share = get_package_share_directory('f1tenth_gym_ros')
    ftg_share = get_package_share_directory('ftg_rv')

    sim_config = os.path.join(gym_share, 'config', 'sim.yaml')
    # Ruta SIN extension: el bridge le agrega '.yaml' (gym) y map_img_ext (png).
    obs_map_path = os.path.join(ftg_share, 'maps', 'SaoPaulo_obs_map')

    # Numero de agentes como argumento de launch: '1' = solo el ego (Parte 2
    # basica), '2' = ego + oponente (lo usa controller_opp.launch.py). El
    # bridge del profesor solo soporta 1 o 2.
    num_agent_arg = DeclareLaunchArgument('num_agent', default_value='1')
    num_agent = LaunchConfiguration('num_agent')
    has_opp = IfCondition(PythonExpression(["'", num_agent, "' == '2'"]))

    # Overrides sobre el sim.yaml del profesor (la ultima fuente gana):
    # - map_path: el mapa con obstaculos de este paquete.
    # - stheta: con los obstaculos, la orientacion inicial de la Parte 1 (0 rad)
    #   deja el primer obstaculo de frente y el auto choca nada mas salir; se
    #   gira el spawn para que arranque apuntando a pista libre.
    # - num_agent: viene del argumento de launch (el yaml del profesor fija 1).
    # - sx1/sy1/stheta1: pose inicial del oponente. La del sim.yaml (9.5, 8.5)
    #   es de otro mapa y en SaoPaulo cae fuera de la pista; se usa una pose
    #   sobre la linea de carrera, mas adelante que el ego. Solo tiene efecto
    #   con num_agent=2.
    obs_overrides = {
        'map_path': obs_map_path,
        'stheta': math.radians(-60.0),
        'num_agent': ParameterValue(num_agent, value_type=int),
        'sx1': 38.09,
        'sy1': -8.26,
        'stheta1': 1.326,
    }
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[sim_config, obs_overrides],
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
        condition=has_opp,      # solo se lanza con num_agent=2
    )

    ld = LaunchDescription()
    ld.add_action(num_agent_arg)
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(opp_robot_publisher)

    return ld
