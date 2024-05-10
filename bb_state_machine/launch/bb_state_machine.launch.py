from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    bbsm_dir = get_package_share_directory('bb_state_machine')

    return LaunchDescription([
        Node(
            package='bb_state_machine',
            executable='bb_state_machine_node',
            name='bb_state_machine_node',
            parameters=[{'data_path': bbsm_dir + '/config'}],
            output='screen'  # Directs the node's output to the console.
        )
    ])
