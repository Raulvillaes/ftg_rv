import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ftg_rv'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Instala launch/ y config/ en el share del paquete para que
        # ros2 launch y get_package_share_directory los encuentren.
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Mapas SaoPaulo (limpio y con los obstaculos fijos de la Parte 2):
        # PNG + yaml de metadatos juntos para que map_path los resuelva. Al
        # viajar en el share del paquete, no hay que tocar el sim.yaml del
        # simulador para cambiar de mapa.
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raulvillaes',
    maintainer_email='raulvillaes@outlook.com',
    description='Controlador reactivo Follow the Gap para el simulador '
                'F1Tenth, con contador de vueltas y cronometro por vuelta.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'reactive_node = ftg_rv.reactive_node:main',
            'lap_timer_node = ftg_rv.lap_timer_node:main',
        ],
    },
)
