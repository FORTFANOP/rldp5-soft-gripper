from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("lidar_simulation")

    world_file = os.path.join(
        package_share,
        "worlds",
        "lidar_test.sdf",
    )

    bridge_config = os.path.join(
        package_share,
        "config",
        "lidar_bridge.yaml",
    )

    rviz_config = os.path.join(
        package_share,
        "rviz",
        "lidar.rviz",
    )

    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz",
        ),

        # Start Gazebo using the installed ros_gz_sim launch file.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ),
            launch_arguments={
                "gz_args": ["-r ", world_file],
            }.items(),
        ),

        # Bridge the simulated point cloud into ROS 2.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="lidar_bridge",
            arguments=["--ros-args", "-p", f"config_file:={bridge_config}"],
            output="screen",
        ),

        # Optionally start RViz.
        Node(
            package="rviz2",
            executable="rviz2",
            name="lidar_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
