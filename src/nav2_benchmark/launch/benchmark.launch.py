import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

# navigation2.launch.py reads TURTLEBOT3_MODEL via os.environ at module level (not inside
# generate_launch_description), so it must be set here before that file is ever imported.
os.environ['TURTLEBOT3_MODEL'] = 'waffle'


def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_benchmark')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    controller = LaunchConfiguration('controller')
    world      = LaunchConfiguration('world')
    map_yaml   = LaunchConfiguration('map')
    x_pose     = LaunchConfiguration('x_pose')
    y_pose     = LaunchConfiguration('y_pose')

    params_file = [pkg_share + '/config/nav2_', controller, '.yaml']

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),

        # Make the bundled models (sun, ground_plane, maze2, …) visible to Gazebo
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            [os.path.join(pkg_share, 'worlds', 'include'),
             ':', EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')],
        ),

        DeclareLaunchArgument(
            'controller',
            default_value='dwb',
            choices=['dwb', 'mppi', 'rpp'],
            description='Nav2 local controller: dwb | mppi | rpp',
        ),

        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                pkg_tb3_gazebo, 'worlds', 'turtlebot3_world.world'),
            description='Full path to a Gazebo .world file',
        ),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'map', 'map.yaml'),
            description='Full path to a Nav2 map .yaml file',
        ),

        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='Robot spawn X (metres)',
        ),

        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='Robot spawn Y (metres)',
        ),

        LogInfo(msg=['[nav2_benchmark] params_file resolved to: ', *params_file]),

        # --- Gazebo ---
        # turtlebot3_world.launch.py hardcodes the world path, so we replicate
        # its four sub-includes here to make world configurable.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
            }.items(),
        ),

        # --- Nav2 stack + RViz ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'launch', 'navigation2.launch.py')),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': 'True',
                'params_file': params_file,
                'initial_pose_x': x_pose,  # <-- ADD THIS LINE
                'initial_pose_y': y_pose,

            }.items(),
        ),

        # --- Keyboard teleop in its own xterm (needs a real TTY for stdin) ---
        ExecuteProcess(
            cmd=['xterm', '-T', 'teleop_keyboard', '-e',
                 'ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'],
            output='screen',
        ),
    ])
