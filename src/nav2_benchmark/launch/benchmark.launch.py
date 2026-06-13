import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share    = get_package_share_directory('nav2_benchmark')
    tb3_gazebo   = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    tb3_nav2     = get_package_share_directory('turtlebot3_navigation2')

    controller = LaunchConfiguration('controller')
    declare_controller = DeclareLaunchArgument('controller', default_value='dwb')

    params_path = [os.path.join(pkg_share, 'config'), '/nav2_', controller, '.yaml']
    map_path    = os.path.join(pkg_share, 'maps', 'benchmark.yaml')
    rviz_config = os.path.join(tb3_nav2, 'rviz', 'tb3_navigation2.rviz')

    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')),
    )

    nav2 = TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'use_sim_time': 'True',
                'autostart':    'True',
                'map':          map_path,
                'params_file':  params_path,
            }.items()
        )
    ])

    rviz = TimerAction(period=7.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        )
    ])

    return LaunchDescription([set_model, declare_controller, gazebo, nav2, rviz])
