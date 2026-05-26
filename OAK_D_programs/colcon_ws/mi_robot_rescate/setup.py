from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mi_robot_rescate'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Registra el paquete en el índice de ROS 2 para que 'ros2 run' lo encuentre
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Incluye el package.xml en la carpeta de instalación
        ('share/' + package_name, ['package.xml']),
        # Si tienes archivos de lanzamiento (launch), descomenta la siguiente línea:
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Si tienes archivos de parámetros (config), descomenta la siguiente línea:
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrés',
    maintainer_email='andres@todo.com',
    description='Paquete de visión y tracking para robot de rescate en incendios',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_rescate = mi_robot_rescate.detector_rescate:main',
            'filtro_humo = mi_robot_rescate.filtro_humo:main',
        ],
    },
)