from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='transformer',
        #     executable='lidar_tf',
        #     name='lidar_tf',
        #     output='screen',
        # ),
        # Node(
        #     package='transformer',
        #     executable='point_cloud_transformer',
        #     name='point_cloud_transformer',
        #     output='screen',
        # ),
        Node(
            package='transformer',
            executable='zed_tf',
            name='zed_tf',
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom_tf',
            arguments=['0.165', '0', '0.475', '0', '0', '0', '1', 'base_link', 'os_sensor'],
            output='screen',
        ),
    ])
