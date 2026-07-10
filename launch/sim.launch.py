"""Launch del simulador con los mapas SaoPaulo de este paquete.

El launch del simulador (gym_bridge_launch.py) lee siempre su config/sim.yaml,
cuyo map_path hay que editar a mano (y recompilar) para cambiar de mapa. Para
evitar ese paso sin modificar su repo, este launch replica sus mismos nodos
(bridge, map_server, lifecycle, RViz, robot_state_publisher) pero sobreescribe
los parametros propios de cada escenario; ambos mapas (limpio y con
obstaculos) viajan instalados en el share de MI paquete (ftg_rv/maps/).

El resto de parametros (topicos, etc.) se sigue leyendo del sim.yaml del
simulador: unica fuente de verdad para todo lo que no depende del escenario.
En launch_ros, cuando un nodo recibe varias fuentes de parametros, la ultima
gana, asi que [sim.yaml, overrides] aplica solo esos overrides.

Argumentos:
- map: nombre del mapa en ftg_rv/maps/, sin extension. Por defecto
  'SaoPaulo_map' (limpio, Parte 1); 'SaoPaulo_obs_map' para la Parte 2.
- stheta: orientacion inicial del ego en radianes. Por defecto 0.0 (la del
  sim.yaml del profesor); el escenario con obstaculos la gira para no
  arrancar de frente al primer obstaculo.
- num_agent: '1' = solo el ego, '2' = ego + oponente. OJO: con num_agent=2
  el simulador no avanza hasta que ALGUIEN publique en /opp_drive, asi que
  '2' se usa desde controller_opp.launch.py, que lanza los dos controladores.

Uso:
    ros2 launch ftg_rv sim.launch.py                            # mapa limpio
    ros2 launch ftg_rv sim.launch.py map:=SaoPaulo_obs_map      # obstaculos
"""
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

    map_arg = DeclareLaunchArgument('map', default_value='SaoPaulo_map')
    stheta_arg = DeclareLaunchArgument('stheta', default_value='0.0')
    num_agent_arg = DeclareLaunchArgument('num_agent', default_value='1')

    # Ruta al mapa SIN extension: el bridge le agrega '.yaml' (gym) y
    # map_img_ext (png). Concatenar [prefijo, sustitucion] produce la ruta
    # completa una vez resuelto el argumento.
    maps_prefix = os.path.join(ftg_share, 'maps') + os.sep
    map_path = [maps_prefix, LaunchConfiguration('map')]

    num_agent = LaunchConfiguration('num_agent')
    has_opp = IfCondition(PythonExpression(["'", num_agent, "' == '2'"]))

    # Overrides sobre el sim.yaml del simulador (la ultima fuente gana):
    # - map_path: el mapa pedido por argumento, instalado en este paquete.
    # - stheta: orientacion inicial del ego (argumento; en radianes).
    # - num_agent: viene del argumento de launch (el yaml del simulador fija 1).
    # - sx1/sy1/stheta1: pose inicial del oponente. La del sim.yaml (9.5, 8.5)
    #   es de otro mapa y en SaoPaulo cae fuera de la pista; se usa una pose
    #   sobre la linea de carrera, mas adelante que el ego. Solo tiene efecto
    #   con num_agent=2.
    scenario_overrides = {
        'map_path': map_path,
        'stheta': ParameterValue(LaunchConfiguration('stheta'),
                                 value_type=float),
        'num_agent': ParameterValue(num_agent, value_type=int),
        'sx1': 38.09,
        'sy1': -8.26,
        'stheta1': 1.326,
    }
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[sim_config, scenario_overrides],
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
        parameters=[{'yaml_filename': [maps_prefix,
                                       LaunchConfiguration('map'), '.yaml']},
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
    ld.add_action(map_arg)
    ld.add_action(stheta_arg)
    ld.add_action(num_agent_arg)
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(opp_robot_publisher)

    return ld
