#!/usr/bin/python3

from pathlib import Path
import re
import time
import yaml

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

from moveit.utils import create_params_file_from_dict
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder


def main():
    rclpy.init()

    reader = rclpy.create_node("moveit_py_description_reader")
    robot = None

    try:
        # Read the descriptions published by the running simulation.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        descriptions = {}

        def receive_urdf(msg):
            descriptions["robot_description"] = msg.data

        def receive_srdf(msg):
            descriptions["robot_description_semantic"] = msg.data

        subscriptions = [
            reader.create_subscription(
                String,
                "/robot_description",
                receive_urdf,
                qos,
            ),
            reader.create_subscription(
                String,
                "/robot_description_semantic",
                receive_srdf,
                qos,
            ),
        ]

        deadline = time.monotonic() + 10.0

        while (
            len(descriptions) < 2
            and time.monotonic() < deadline
            and rclpy.ok()
        ):
            rclpy.spin_once(reader, timeout_sec=0.1)

        if len(descriptions) < 2:
            raise RuntimeError(
                "Could not retrieve both robot descriptions. "
                f"Received: {list(descriptions.keys())}"
            )

        print("Received URDF and SRDF from the running system.")

        # Build the installed UR MoveIt configuration.
        config = (
            MoveItConfigsBuilder(
                robot_name="ur",
                package_name="ur_moveit_config",
            )
            .robot_description_semantic(
                Path("srdf") / "ur.srdf.xacro",
                {"name": "ur"},
            )
            .trajectory_execution(
                file_path="config/moveit_controllers.yaml"
            )
            .to_moveit_configs()
            .to_dict()
        )

        # Use the actual descriptions from the running simulation.
        config.update(descriptions)

        # Make the SRDF robot name match the URDF robot name.
        # Only the root <robot name="..."> attribute is changed.
        srdf = config["robot_description_semantic"]

        srdf, replacements = re.subn(
            r'(<robot\b[^>]*\bname=["\'])ur5e(["\'])',
            r'\g<1>ur\g<2>',
            srdf,
            count=1,
        )

        if replacements == 1:
            config["robot_description_semantic"] = srdf

        # Verify the actual descriptions before initializing MoveItPy.
        robot_names = {}

        for key in (
            "robot_description",
            "robot_description_semantic",
        ):
            match = re.search(
                r'<robot\b[^>]*\bname=["\']([^"\']+)',
                config[key],
            )

            if not match:
                raise RuntimeError(
                    f"Could not determine the robot name in {key}."
                )

            robot_names[key] = match.group(1)
            print(f"{key}: {match.group(1)}")

        if (
            robot_names["robot_description"]
            != robot_names["robot_description_semantic"]
        ):
            raise RuntimeError(
                "URDF/SRDF robot-name mismatch: "
                f"{robot_names['robot_description']} vs "
                f"{robot_names['robot_description_semantic']}"
            )

        # Load Python-specific planning settings.
        config_dir = (
            Path.home()
            / "robotic-arm-project/lidar_ws/src/ur_lidar_bringup/config"
        )

        with open(config_dir / "moveit_py.yaml", "r") as f:
            python_config = yaml.safe_load(f)

        config.update(python_config)

        # Explicitly keep only the active trajectory controller.
        config["moveit_simple_controller_manager"] = (
            python_config["moveit_simple_controller_manager"]
        )

        print("Initializing MoveItPy with simulation time enabled...")

        # Scope parameters globally to avoid the MoveItPy QoS override issue.
        params_file = create_params_file_from_dict(config, "/**")

        robot = MoveItPy(
            node_name="moveit_py",
            launch_params_filepaths=[params_file],
        )

        print("MoveItPy initialized successfully.")
        print("Waiting for trajectory controller discovery...")
        time.sleep(3.0)
        model = robot.get_robot_model()
        print("Robot model loaded:", model)

        arm = robot.get_planning_component("ur_manipulator")
        print("Planning group:", arm.planning_group_name)
        print("Named target states:", list(arm.named_target_states))

        # Test actual motion planning
        print("\nTesting motion planning...")

        arm.set_start_state_to_current_state()
        arm.set_goal_state(configuration_name="up")

        print("Planning from the current state to the named state 'up'...")

        plan_result = arm.plan()

        if plan_result:
            print("\nSUCCESS: MoveItPy generated a trajectory.")

            trajectory = plan_result.trajectory
            print("Trajectory object:", trajectory)

            # print("Planning test passed. The robot has NOT been commanded to move.")
            print("Executing trajectory in simulation...")
            robot.execute(trajectory, controllers=[])
            print("Trajectory execution command completed.")

        else:
            print("\nPLANNING FAILED.")
            print("Planning failed. Nothing will be executed.")



    finally:
        if robot is not None:
            robot.shutdown()

        reader.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()