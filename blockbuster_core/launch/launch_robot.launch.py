import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition

from launch_ros.actions import Node



def generate_launch_description():

    activate_slam_arg = DeclareLaunchArgument('activate_slam', default_value='false', description='Flag to activate SLAM')
    activate_nav_arg = DeclareLaunchArgument('activate_nav', default_value='false', description='Flag to activate Nav2')
    activate_loc_arg = DeclareLaunchArgument('activate_loc', default_value='false', description='Flag to activate localisation')
    activate_cam_arg = DeclareLaunchArgument('activate_cam', default_value='false', description='Flag to activate camera')
    activate_sm_arg = DeclareLaunchArgument('activate_sm', default_value='false', description='Flag to activate sm')

    package_name='blockbuster_core'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    
    rplidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rplidar.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    storage_servo_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["storage_servo"],
    )

    delayed_storage_servo_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[storage_servo_spawner],
        )
    )

    system_interface_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fts_broadcaster"],
    )

    delayed_system_interface_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[system_interface_spawner],
        )
    )

    yolov6_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('oakd_cam'), 'launch', 'yolov6_publisher.launch.py')]),
        condition=IfCondition(LaunchConfiguration('activate_cam'))
    )

    delayed_yolov6_launch = TimerAction(
        period=3.0, 
        actions=[yolov6_launch]
    )

    slam_toolbox_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory('blockbuster_core'), 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'false'
        }.items(),
        condition=IfCondition(LaunchConfiguration('activate_slam'))
    )

    delayed_slam_launch = TimerAction(
        period=5.0, 
        actions=[slam_toolbox_launch_description]
    )

    loc_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory('blockbuster_core'), 'config', 'nav2_params.yaml'),
            'map':os.path.join(get_package_share_directory('blockbuster_core'), 'maps', 'map_arena_test.yaml'),
            'use_sim_time': 'false'
        }.items(),
        condition=IfCondition(LaunchConfiguration('activate_loc'))
    )

    delayed_loc_launch = TimerAction(
        period=5.0, 
        actions=[loc_launch_description]
    )

    cost_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('blockbuster_core'), 'launch', 'costmap_filter_info.launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory('blockbuster_core'), 'config', 'costmap_params.yaml'),
            'mask':os.path.join(get_package_share_directory('blockbuster_core'), 'maps', 'map_arena_test_keepout.yaml'),
            'use_sim_time': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('activate_nav'))
    )

    delayed_cost_launch = TimerAction(
        period=12.0, 
        actions=[cost_launch_description]
    )
    
    nav_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory('blockbuster_core'), 'config', 'nav2_params.yaml'),
            'use_sim_time': 'false',
            'map_subscribe_transient_local': 'true',
            'log_level': 'error'
        }.items(),
        condition=IfCondition(LaunchConfiguration('activate_nav'))
    )

    delayed_nav_launch = TimerAction(
        period=15.0, 
        actions=[nav_launch_description]
    )

    sm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('blockbuster_sm'), 'launch', 'blockbuster_sm.launch.py')
        ]),
        condition=IfCondition(LaunchConfiguration('activate_sm'))
    )

    delayed_sm_launch = TimerAction(
        period=16.0, 
        actions=[sm_launch]
    )

    return LaunchDescription([
        rsp,
        twist_mux,
        rplidar,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        delayed_storage_servo_spawner,
        delayed_system_interface_spawner,
        activate_cam_arg,
        delayed_yolov6_launch,
        activate_slam_arg,
        delayed_slam_launch,
        activate_nav_arg,
        delayed_nav_launch,
        activate_loc_arg,
        delayed_loc_launch,
        delayed_cost_launch,
        activate_sm_arg,
        delayed_sm_launch
    ])
