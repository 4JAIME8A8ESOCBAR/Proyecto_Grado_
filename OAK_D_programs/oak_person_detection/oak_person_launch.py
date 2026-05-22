from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='oak_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depthai_ros_driver_v3',
                    plugin='depthai_ros_driver::Camera',
                    name='oak',
                    parameters=[{
                        'i_pipeline_type': 'RGBD',
                        'i_nn_type': 'yolov8',
                        'i_nn_model': 'yolov8n_coco_640x352',
                        'i_enable_spatial_detection': True,
                        'nn.i_label_filter': ['person'],
                        'i_enable_ir': True,
                        'ir.i_led_brightness': 0,
                    }]
                )
            ]
        )
    ])
