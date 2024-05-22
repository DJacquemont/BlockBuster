from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    bbsm_dir = get_package_share_directory('bb_state_machine')
    articubot_one_dir = get_package_share_directory('articubot_one')

    return LaunchDescription([
        Node(
            package='bb_state_machine',
            executable='bb_state_machine_node',
            name='bb_state_machine_node',
            parameters=[
                {'data_path': os.path.join(bbsm_dir, 'config'),
                 'map_1': os.path.join(articubot_one_dir, 'maps', 'map_1.yaml'),
                 'map_2': os.path.join(articubot_one_dir, 'maps', 'map_2.yaml')}
            ],
            output='screen'  # Directs the node's output to the console.
        )
    ])
