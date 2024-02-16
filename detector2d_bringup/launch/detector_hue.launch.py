import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # ComposableNode(
            #     package='realsense2_camera',
            #     plugin='realsense2_camera::RealSenseNodeFactory',
            #     name='realsense2_camera',
            #     namespace='',
            #     parameters=[{
            #         'align_depth.enable': True,
            #         'enable_color': True,
            #         'enable_depth': True
            #     }]
            # ),
            ComposableNode(
                package='detector2d_node',
                plugin='detector2d_node::Detector2dNode',
                name='detector2d_node',
                namespace='',
                parameters=[{
                    'load_target_plugin': 'detector2d_plugins::PanelSimpleDetector',
                    'debug': True
                }],
                remappings=[
                    ('image_raw', 'blue/camera/image_raw')
                ]
            )
        ]
    )

    return LaunchDescription([
        container
    ])
