from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define arguments for the included launch files
    sensor_hostname = LaunchConfiguration('sensor_hostname')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    # Declare arguments with default values
    declare_sensor_hostname = DeclareLaunchArgument(
        'sensor_hostname', default_value='os-122220002210.local'
    )
    declare_pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic', default_value='/ouster/points'
    )
    declare_imu_topic = DeclareLaunchArgument(
        'imu_topic', default_value='/ouster/imu'
    )

    # Get package directories
    ouster_ros_dir = get_package_share_directory('ouster_ros')
    direct_lidar_odometry_dir = get_package_share_directory('direct_lidar_odometry')
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    transformer_dir = get_package_share_directory('transformer')

    # Include launch files with arguments
    driver_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'ouster_ros', 'sensor.launch.xml', 'sensor_hostname:=os-122220002210.local'],
        output='screen'
    )

    dlo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(direct_lidar_odometry_dir, 'launch', 'dlo.launch.py')),
        launch_arguments={
            'pointcloud_topic': pointcloud_topic,
            'imu_topic': imu_topic
        }.items()
    )

    zed_no_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')),
        launch_arguments={
            "camera_model": "zed2i"
        }.items()
    )

    transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(transformer_dir, 'launch', 'transform.launch.py'))
    )

    # mono_cam = Node(
    #     package='vision',
    #     executable='mono_cam.py',
    #     name='mono_cam',
    #     output='screen',
    # )

    return LaunchDescription([
        declare_sensor_hostname,
        declare_pointcloud_topic,
        declare_imu_topic,
        driver_launch,
        dlo_launch,
        zed_no_tf_launch,
        transform_launch,
        # mono_cam
    ])
