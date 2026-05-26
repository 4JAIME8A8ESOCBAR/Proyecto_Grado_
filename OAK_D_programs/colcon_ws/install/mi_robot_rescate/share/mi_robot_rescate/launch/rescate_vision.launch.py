import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Buscar la ruta de nuestro archivo de parámetros personalizado
    pkg_dir = get_package_share_directory('mi_robot_rescate')
    config_path = os.path.join(pkg_dir, 'config', 'params.yaml')

    # Creamos el contenedor y cargamos el Driver con nuestro archivo YAML directamente
    container = ComposableNodeContainer(
        name='oak_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depthai_ros_driver_v3',
                plugin='depthai_ros_driver::Driver',
                name='oak',
                parameters=[config_path] # Forzado nativo de ROS 2
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])