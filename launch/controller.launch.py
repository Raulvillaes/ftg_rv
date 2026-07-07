"""Launch del controlador Follow the Gap.

Lanza reactive_node cargando los parametros desde config/params.yaml,
de forma que se puedan tunear sin tocar el codigo.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # Ruta al params.yaml ya instalado en el share del paquete
    params_file = os.path.join(
        get_package_share_directory('ftg_rv'),
        'config',
        'params.yaml',
    )

    reactive_node = Node(
        package='ftg_rv',
        executable='reactive_node',
        name='reactive_node',
        parameters=[params_file],
        output='screen',        # logs visibles en la consola (video de evidencia)
        emulate_tty=True,       # conserva colores/formato del logger de ROS 2
    )

    # El launch del simulador tiene una condicion de carrera: su
    # lifecycle_manager intenta configurar el map_server antes de que este
    # listo y aborta, dejando a RViz sin mapa ("No map received"). Para no
    # modificar el repositorio del simulador, reintento la activacion desde
    # aqui. Si el map_server ya esta activo, las transiciones fallan sin
    # efecto (por eso el "|| true").
    activate_map_server = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=['bash', '-c',
                 'ros2 lifecycle set /map_server configure || true; '
                 'ros2 lifecycle set /map_server activate || true'],
            output='log',
        )],
    )

    return LaunchDescription([reactive_node, activate_map_server])
