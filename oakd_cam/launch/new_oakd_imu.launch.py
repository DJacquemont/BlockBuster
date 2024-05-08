import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    
    depthai_examples_path = get_package_share_directory('depthai_examples')
    oakd_path = get_package_share_directory('oakd_cam')

    default_resources_path = os.path.join(oakd_path, 'resources')

    mxId         = LaunchConfiguration('mxId',      default = 'x')
    usb2Mode     = LaunchConfiguration('usb2Mode',  default = False)
    poeMode      = LaunchConfiguration('poeMode',   default = False)
    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    tf_prefix    = LaunchConfiguration('tf_prefix',     default = 'oak')
    base_frame   = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')
    imuMode      = LaunchConfiguration('imuMode', default = '1')
    nnName                  = LaunchConfiguration('nnName', default = 'x')
    resourceBaseFolder      = LaunchConfiguration('resourceBaseFolder', default = default_resources_path)
    angularVelCovariance  = LaunchConfiguration('angularVelCovariance', default = 0.02)
    linearAccelCovariance = LaunchConfiguration('linearAccelCovariance', default = 0.02)
    enableRosBaseTimeUpdate       = LaunchConfiguration('enableRosBaseTimeUpdate', default = False)

    stereo_node = launch_ros.actions.Node(
            package='oakd_cam', executable='oakd_imu_node',
            output='screen',
            parameters=[{'mxId':                    mxId},
                        {'usb2Mode':                usb2Mode},
                        {'poeMode':                 poeMode},
                        {'resourceBaseFolder':      resourceBaseFolder},
                        {'tf_prefix':               tf_prefix},
                        {'imuMode':                 imuMode},
                        {'angularVelCovariance':    angularVelCovariance},
                        {'linearAccelCovariance':   linearAccelCovariance},
                        {'nnName':                  nnName},
                        {'enableRosBaseTimeUpdate': enableRosBaseTimeUpdate}
                        ])

    ld = LaunchDescription()

    ld.add_action(stereo_node)
    
    return ld
