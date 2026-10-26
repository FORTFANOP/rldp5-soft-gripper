import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # Locate the installed package directories.
    ur_sim_share = get_package_share_directory("ur_simulation_gz")
    moveit_share = get_package_share_directory("ur_moveit_config")
    lidar_share = get_package_share_directory("lidar_simulation")

    # Reuse our existing LiDAR world and bridge configuration.
    world_file = os.path.join(
        lidar_share, "worlds", "lidar_test.sdf"
    )

    bridge_config = os.path.join(
        lidar_share, "config", "lidar_bridge.yaml"
    )

    # 1. Start the UR5e simulation in our custom Gazebo world.
    robot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "ur_sim_control_raised.launch.py",
            )
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "world_file": world_file,
            "launch_rviz": "false",
        }.items(),
    )

    # 2. Start MoveIt using the installed UR5e configuration.
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_share,
                "launch",
                "ur_moveit.launch.py",
            )
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "use_sim_time": "true",
            "launch_rviz": "true",
        }.items(),
    )

    # 3. Bridge the simulated LiDAR point cloud into ROS 2.
    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="lidar_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_config}",
        ],
        output="screen",
    )

    # 4. Launch the saved LiDAR visualization in a separate RViz window
    lidar_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="lidar_rviz",
        arguments=[
            "-d",
            os.path.join(lidar_share, "rviz", "lidar.rviz"),
        ],
        parameters=[
            {"use_sim_time": True},
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_simulation,
        moveit,
        lidar_bridge,
        lidar_rviz,
    ])
