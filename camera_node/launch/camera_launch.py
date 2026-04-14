from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='camera',
                plugin='sensor::CameraNode',
                name='camera_node',
                parameters=[{
                    'videoflag': True,
                    'video_path': '/home/phoenix/zk/save_stuff/20.mp4',
                    'inner_shot_flag': False,
                    'rosbag_flag': False,
                    'exposure_time': 10000,
                    'gain': 64
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])
