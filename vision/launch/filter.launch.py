from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='lane_filterer.py',
            name='lane_filter_node',
            output='screen',
            parameters=[
                {'higher_h': 180},
                {'higher_s': 42},
                {'higher_v': 255},
                {'lower_h': 32},
                {'lower_s': 0},
                {'lower_v': 190},
                {'kernel_size': 3},
                {'iterations': 20}
            ],
            remappings=[
                ('rgb_image_topic_sub', 'zed/zed_node/rgb/image_rect_color'),
                ('mask_topic_pub', '/lane_filter/mask')
            ]
        )
    ])
