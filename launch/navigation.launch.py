from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_nav_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    log_level = LaunchConfiguration('log_level')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_namespace': 'false',
            'rviz_config': rviz_config,
        }.items(),
        condition=IfCondition(use_rviz),
    )

    # ENU: right turn around +Z is negative yaw.
    laser_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        output='screen',
        arguments=[
            '0.395', '0.0', '0.0',
            '-0.78539816339', '0.0', '0.0',
            'base_link', 'laser_link',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'generated_map.yaml'),
            description='Full path to the map YAML file.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Full path to the Nav2 parameters file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically transition Nav2 lifecycle nodes.',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Keep false for a simpler minimal bringup.',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Respawn Nav2 nodes if they crash.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz together with Nav2.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz'),
            description='Full path to the RViz config file.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level for Nav2 nodes.',
        ),
        localization_launch,
        navigation_launch,
        rviz_launch,
        laser_static_tf,
    ])
