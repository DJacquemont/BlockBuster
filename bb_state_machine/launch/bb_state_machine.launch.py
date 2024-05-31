from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    bbsm_dir = get_package_share_directory('bb_state_machine')
    blockbuster_core_dir = get_package_share_directory('blockbuster_core')

    return LaunchDescription([
        Node(
            package='bb_state_machine',
            executable='bb_state_machine_node',
            name='bb_state_machine_node',
            parameters=[
                {'data_path': os.path.join(bbsm_dir, 'config'),
                 'map_0': os.path.join(blockbuster_core_dir, 'maps', 'map_arena_l0.yaml'),
                 'map_1': os.path.join(blockbuster_core_dir, 'maps', 'map_arena_l1.yaml')}
            ],
            output='screen'
        )
    ])
