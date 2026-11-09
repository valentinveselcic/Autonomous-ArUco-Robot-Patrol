from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments exposed by *your* launch file
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rosbot3_nav"), "config", "slam.yaml"]
        ),
        description="Full path to the slam_toolbox parameters file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )

    # Path to slam_toolbox's online_async_launch.py
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
            )
        ),
        launch_arguments={
            # Forward arguments to slam_toolbox's launch file
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        slam_params_file_arg,
        use_sim_time_arg,
        slam_toolbox_launch,
    ])