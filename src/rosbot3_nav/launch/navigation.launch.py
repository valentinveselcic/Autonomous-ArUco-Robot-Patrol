from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    tutorial_dir = FindPackageShare('rosbot3_nav')
    nav2_bringup_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([tutorial_dir, 'maps', 'map.yaml']),
        description='Full path to map yaml file to load',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [tutorial_dir, 'config', 'nav2_jazzy_params.yaml']
        ),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([tutorial_dir, 'rviz', 'navigation.rviz']),
        description='Full path to the RViz config file to use',
    )

    # Original Nav2 bringup include
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch_file_dir]),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            # We start RViz ourselves below
            'use_rviz': 'false',
        }.items(),
    )

    # Group Nav2 under a remap: /cmd_vel -> /cmd_vel_nav (Twist)
    nav2_group = GroupAction(
        [
            SetRemap('/cmd_vel', '/cmd_vel_nav'),
            nav2_bringup_launch,
        ]
    )

    # Bridge node: /cmd_vel_nav (Twist) -> /cmd_vel (TwistStamped)
    cmd_vel_stamper_node = Node(
        package='rosbot3_nav',
        executable='cmd_vel_stamper',
        name='cmd_vel_stamper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # RViz2 with your navigation config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='nav2_rviz',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription(
        [
            declare_map_yaml_cmd,
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            declare_rviz_config_cmd,
            nav2_group,
            cmd_vel_stamper_node,
            rviz_node,
        ]
    )
