from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='pc_mapper_node',
            name='pc_mapper_node',
            output='screen',
            parameters=[
                {'prior': 0.5},
                {'prob_hit': 0.8},
                {'prob_miss': 0.2},
                {'min_prob': 0.05},
                {'max_prob': 0.99},
                {'obstacle_threshold': 0.6},
                {'octree_resolution': 0.01},
                {'resolution': 0.01},
                {'width': 3500},
                {'height': 3500}
            ],
            remappings=[
                ('pointcloud_topic_sub', '/odom/point_cloud'),
                ('odom_topic_sub', '/robot/dlo/odom_node/odom'),
                ('mask_topic_sub', '/lane_filter/mask'),
                ('debug_cloud_pub', '/lane_filter/debug_cloud'),
                ('map_pub', '/global_map')
            ]
        )
    ])
