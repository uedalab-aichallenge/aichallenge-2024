import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # simple_dwa ノードの起動
    simple_dwa = Node(
        package='simple_dwa',
        executable='simple_dwa',
        name='simple_dwa',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('simple_dwa'),
            'config',
            'config.yaml'
        )]
    )
    # state_publisher ノードの起動
    state_publisher = Node(
        package='state_publisher',
        executable='state_publisher',
        name='state_publisher',
        output='screen'
    )

    # rviz の起動
    rviz_config_path = os.path.join(
        get_package_share_directory('simple_dwa'),
        'rviz',
        'dwa.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        simple_dwa,
        state_publisher,
        rviz_node
    ])