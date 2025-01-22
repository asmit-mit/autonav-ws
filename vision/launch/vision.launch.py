from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the directory of the vision package
    vision_package_dir = get_package_share_directory('vision')

    # Paths to the included launch files
    sensors_launch = os.path.join(vision_package_dir, 'launch', 'sensors.launch.py')
    filter_launch = os.path.join(vision_package_dir, 'launch', 'filter.launch.py')
    mapping_launch = os.path.join(vision_package_dir, 'launch', 'mapping.launch.py')

    # Include the other launch files
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(filter_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch)
        ),
        # Add the static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map'],
            output='screen'
        )
    ])
