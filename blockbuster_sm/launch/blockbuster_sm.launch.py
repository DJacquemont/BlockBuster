from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    bbsm_dir = get_package_share_directory('blockbuster_sm')

    return LaunchDescription([
        Node(
            package='blockbuster_sm',
            executable='blockbuster_sm_node',
            name='blockbuster_sm_node',
            parameters=[
                {'data_path': os.path.join(bbsm_dir, 'config')}
            ],
            output='screen'
        )
    ])
