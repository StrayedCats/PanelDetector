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
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='realsense2_camera',
                namespace='',
                parameters=[{
                    'align_depth.enable': True,
                    'enable_color': True,
                    'enable_depth': True
                }]
            ),
            ComposableNode(
                package='detector2d_node',
                plugin='detector2d_node::Detector2dNode',
                name='detector2d_node',
                namespace='',
                parameters=[{
                    'yolox_trt_plugin.model_path': '/models/yolox_s.trt',
                    'load_target_plugin': 'detector2d_plugins::YoloxTrt'
                }],
                remappings=[
                    ('image_raw', 'realsense2_camera/color/image_raw')
                ]
            ),
            ComposableNode(
                package='bbox2d_to_3d_node',
                plugin='bbox2d_to_3d_node::BBox2DTo3DNode',
                name='bbox2d_to_3d_node',
                namespace='',
                parameters=[{
                    'min_depth': 0.05,
                    'max_depth': 3.0,
                    'imshow_isshow': True
                }],
                remappings=[
                    ('bbox2d', 'positions'),
                    ('camera_info', 'realsense2_camera/aligned_depth_to_color/camera_info'),
                    ('depth', 'realsense2_camera/aligned_depth_to_color/image_raw'),
                    ('color', 'realsense2_camera/color/image_raw')
                ]
            ),
        ]
    )

    return LaunchDescription([
        container
    ])
