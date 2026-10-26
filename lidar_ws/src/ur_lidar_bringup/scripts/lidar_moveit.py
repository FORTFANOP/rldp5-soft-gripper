#!/usr/bin/python3

from pathlib import Path
import re
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from moveit.utils import create_params_file_from_dict
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_configs_utils import MoveItConfigsBuilder

from runtime_link_attacher.srv import AttachLink, DetachLink


# =========================================================
# CONFIGURATION
# =========================================================

PREGRASP_CLEARANCE = 0.15  # 15 cm above the detected box top
GRASP_CLEARANCE = 0.10     # 10 cm above the detected box top

# Destination on the opposite side of the robot.
# The default mirrors the detected box's X coordinate.
# Example: detected x = +0.75 m -> destination x = -0.75 m.
MIRROR_DESTINATION_X = True

# Used only when MIRROR_DESTINATION_X is False.
PLACE_X = -0.75
PLACE_Y = 0.0

# Gazebo model/link names.
ROBOT_MODEL = "ur"
ATTACH_LINK = "wrist_3_link"

BOX_MODEL = "obstacle_box"
BOX_LINK = "box_link"

# Runtime link attacher services.
ATTACH_SERVICE = "/runtime_link_attacher/attach"
DETACH_SERVICE = "/runtime_link_attacher/detach"

# Named joint configuration from the MoveIt SRDF.
HOME_CONFIGURATION = "home"


# =========================================================
# LIDAR DETECTION SUBSCRIBER
# =========================================================

class LidarMoveIt(Node):

    def __init__(self):
        super().__init__("lidar_moveit")

        self.latest_pose = None

        self.subscription = self.create_subscription(
            PoseStamped,
            "/detected_box/top_center",
            self.pose_callback,
            10,
        )

        self.get_logger().info(
            "Subscribed to /detected_box/top_center."
        )

    def pose_callback(self, msg):
        self.latest_pose = msg


# =========================================================
# INITIALISE MOVEITPY
# =========================================================

def load_moveit(robot_reader):

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
        robot_reader.create_subscription(
            String,
            "/robot_description",
            receive_urdf,
            qos,
        ),
        robot_reader.create_subscription(
            String,
            "/robot_description_semantic",
            receive_srdf,
            qos,
        ),
    ]

    # Wait for robot descriptions.
    deadline = time.monotonic() + 10.0

    while (
        len(descriptions) < 2
        and time.monotonic() < deadline
        and rclpy.ok()
    ):
        rclpy.spin_once(robot_reader, timeout_sec=0.1)

    if len(descriptions) < 2:
        raise RuntimeError(
            f"Could not retrieve robot descriptions: {list(descriptions)}"
        )

    # Load the standard UR configuration.
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

    # Use descriptions published by the running robot.
    config.update(descriptions)

    # Match the SRDF root name to the running URDF.
    config["robot_description_semantic"], _ = re.subn(
        r'(<robot\b[^>]*\bname=["\'])ur5e(["\'])',
        r'\g<1>ur\g<2>',
        config["robot_description_semantic"],
        count=1,
    )

    # Load MoveItPy-specific configuration.
    config_dir = (
        Path.home()
        / "robotic-arm-project/lidar_ws/src/ur_lidar_bringup/config"
    )

    with open(config_dir / "moveit_py.yaml", "r") as f:
        python_config = yaml.safe_load(f)

    config.update(python_config)

    config["moveit_simple_controller_manager"] = (
        python_config["moveit_simple_controller_manager"]
    )

    params_file = create_params_file_from_dict(config, "/**")

    robot = MoveItPy(
        node_name="moveit_py_lidar",
        launch_params_filepaths=[params_file],
    )

    # Keep references to the subscriptions alive.
    return robot, subscriptions


# =========================================================
# CALL GAZEBO LINK ATTACHER SERVICES
# =========================================================

def call_link_service(
    node,
    service_type,
    service_name,
    parent_link,
    child_link,
):
    """
    Call a runtime_link_attacher service and return its success status.
    """

    client = node.create_client(
        service_type,
        service_name,
    )

    try:
        node.get_logger().info(
            f"Waiting for service {service_name}..."
        )

        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                f"Service {service_name} is unavailable. "
                "Check that the Gazebo link-attacher plugin loaded."
            )

        request = service_type.Request()
        request.parent_link = parent_link
        request.child_link = child_link

        node.get_logger().info(
            f"Calling {service_name}: "
            f"{parent_link} -> {child_link}"
        )

        future = client.call_async(request)
        deadline = time.monotonic() + 10.0

        while (
            not future.done()
            and time.monotonic() < deadline
            and rclpy.ok()
        ):
            rclpy.spin_once(node, timeout_sec=0.1)

        if not future.done():
            raise RuntimeError(
                f"Timed out waiting for {service_name}."
            )

        response = future.result()

        if response is None:
            raise RuntimeError(
                f"No response received from {service_name}."
            )

        if not response.success:
            node.get_logger().error(
                f"{service_name} failed: {response.message}"
            )
            return False

        node.get_logger().info(
            f"{service_name} succeeded: {response.message}"
        )

        return True

    finally:
        node.destroy_client(client)


def attach_box(node):
    """
    Attach the Gazebo box to the robot's wrist_3_link.
    """

    parent_link = f"{ROBOT_MODEL}::{ATTACH_LINK}"
    child_link = f"{BOX_MODEL}::{BOX_LINK}"

    print("\nAttaching box to the robot in Gazebo...")

    success = call_link_service(
        node,
        AttachLink,
        ATTACH_SERVICE,
        parent_link,
        child_link,
    )

    if success:
        print("Box attached successfully.")

    return success


def detach_box(node):
    """
    Detach the Gazebo box from the robot's wrist_3_link.
    """

    parent_link = f"{ROBOT_MODEL}::{ATTACH_LINK}"
    child_link = f"{BOX_MODEL}::{BOX_LINK}"

    print("\nDetaching box in Gazebo...")

    success = call_link_service(
        node,
        DetachLink,
        DETACH_SERVICE,
        parent_link,
        child_link,
    )

    if success:
        print("Box detached successfully.")

    return success


# =========================================================
# PLAN AND OPTIONALLY EXECUTE A POSE
# =========================================================

def plan_and_execute(
    robot,
    arm,
    target,
    description,
):
    """
    Plan to a Cartesian pose and ask for confirmation before execution.

    Returns True only when the trajectory was successfully executed.
    """

    arm.set_start_state_to_current_state()

    arm.set_goal_state(
        pose_stamped_msg=target,
        pose_link="tool0",
    )

    print(f"\nPlanning: {description}")

    # Give OMPL more time and more attempts to find a solution.
    plan_params = PlanRequestParameters(robot, "ompl")
    plan_params.planning_pipeline = "ompl"
    plan_params.planner_id = "RRTConnectkConfigDefault"
    plan_params.planning_time = 15.0
    plan_params.planning_attempts = 10

    result = None

    for attempt in range(1, 4):
        print(
            f"\nPlanning attempt {attempt}/3 "
            f"(up to 15 seconds, 10 planner attempts)..."
        )

        result = arm.plan(
            single_plan_parameters=plan_params
        )

        if result:
            print(f"Planning succeeded on request {attempt}/3.")
            break

        print(f"Request {attempt}/3 failed.")

    if not result:
        print("Planning failed after 3 requests.")
        return False

    if not result:
        print(f"PLANNING FAILED: {description}")
        print("The robot was not commanded to move.")
        return False

    print(f"\nSuccessfully planned: {description}")
    print("Review the planned trajectory in RViz.")

    try:
        confirmation = input(
            f"\nExecute '{description}'? Type 'y' to proceed: "
        ).strip().lower()

    except (KeyboardInterrupt, EOFError):
        print("\nExecution cancelled.")
        return False

    if confirmation != "y":
        print("Execution cancelled.")
        return False

    print("\nExecuting trajectory...")

    success = robot.execute(
        result.trajectory,
        controllers=[],
    )

    if success:
        print(f"Execution completed: {description}.")
        return True

    print(f"Execution failed: {description}.")
    return False


# =========================================================
# PLAN AND OPTIONALLY EXECUTE A NAMED CONFIGURATION
# =========================================================

def plan_and_execute_named(
    robot,
    arm,
    configuration_name,
    description,
):
    """
    Plan to a named SRDF joint configuration, such as 'home'.
    """

    arm.set_start_state_to_current_state()

    arm.set_goal_state(
        configuration_name=configuration_name,
    )

    print(f"\nPlanning: {description}")

    result = arm.plan()

    if not result:
        print(f"PLANNING FAILED: {description}")
        return False

    print(f"\nSuccessfully planned: {description}")

    try:
        confirmation = input(
            f"\nExecute '{description}'? Type 'y' to proceed: "
        ).strip().lower()

    except (KeyboardInterrupt, EOFError):
        print("\nExecution cancelled.")
        return False

    if confirmation != "y":
        print("Execution cancelled.")
        return False

    print("\nExecuting trajectory...")

    success = robot.execute(
        result.trajectory,
        controllers=[],
    )

    if success:
        print(f"Execution completed: {description}.")
        return True

    print(f"Execution failed: {description}.")
    return False


# =========================================================
# MAIN PICK-AND-PLACE SEQUENCE
# =========================================================

def main():

    rclpy.init()

    reader = rclpy.create_node(
        "moveit_py_description_reader"
    )

    lidar_node = LidarMoveIt()

    robot = None

    box_attached = False

    try:

        # -------------------------------------------------
        # 1. INITIALISE MOVEITPY
        # -------------------------------------------------

        robot, description_subscriptions = load_moveit(reader)

        lidar_node.get_logger().info(
            "MoveItPy initialised. Waiting for controller discovery..."
        )

        time.sleep(3.0)

        arm = robot.get_planning_component(
            "ur_manipulator"
        )

        # -------------------------------------------------
        # 2. WAIT FOR LIDAR DETECTION
        # -------------------------------------------------

        print("\nWaiting for a detected box pose...")

        deadline = time.monotonic() + 30.0

        while (
            lidar_node.latest_pose is None
            and time.monotonic() < deadline
            and rclpy.ok()
        ):
            rclpy.spin_once(
                lidar_node,
                timeout_sec=0.2,
            )

        if lidar_node.latest_pose is None:
            raise RuntimeError(
                "No detected box pose received within 30 seconds."
            )

        detected = lidar_node.latest_pose

        if detected.header.frame_id != "world":
            raise RuntimeError(
                "Expected the detected pose in the 'world' frame, "
                f"but received {detected.header.frame_id!r}."
            )

        box_x = detected.pose.position.x
        box_y = detected.pose.position.y
        box_top_z = detected.pose.position.z

        print("\nDetected box top centre:")
        print(f"  x = {box_x:.3f} m")
        print(f"  y = {box_y:.3f} m")
        print(f"  z = {box_top_z:.3f} m")

        # -------------------------------------------------
        # 3. CALCULATE THE DESTINATION
        # -------------------------------------------------

        if MIRROR_DESTINATION_X:
            place_x = -box_x
            place_y = box_y
        else:
            place_x = PLACE_X
            place_y = PLACE_Y

        print("\nPredefined placement location:")
        print(f"  x = {place_x:.3f} m")
        print(f"  y = {place_y:.3f} m")

        # -------------------------------------------------
        # 4. MOVE TO PRE-GRASP
        # -------------------------------------------------

        pregrasp_target = PoseStamped()

        pregrasp_target.header.frame_id = "world"
        pregrasp_target.header.stamp = detected.header.stamp

        pregrasp_target.pose.position.x = box_x
        pregrasp_target.pose.position.y = box_y
        pregrasp_target.pose.position.z = (
            box_top_z + PREGRASP_CLEARANCE
        )

        pregrasp_target.pose.orientation = detected.pose.orientation

        print("\n--- PRE-GRASP DIAGNOSTICS ---")
        print("Target frame:", pregrasp_target.header.frame_id)
        print(
            "Target position:",
            pregrasp_target.pose.position.x,
            pregrasp_target.pose.position.y,
            pregrasp_target.pose.position.z,
        )
        print(
            "Target quaternion:",
            pregrasp_target.pose.orientation.x,
            pregrasp_target.pose.orientation.y,
            pregrasp_target.pose.orientation.z,
            pregrasp_target.pose.orientation.w,
        )
        print("Current robot state will be read by MoveItPy.")
        print("--- END DIAGNOSTICS ---\n")

        pregrasp_success = plan_and_execute(
            robot,
            arm,
            pregrasp_target,
            "pre-grasp approach",
        )

        if not pregrasp_success:
            print("\nStopping: pre-grasp was not completed.")
            return

        # -------------------------------------------------
        # 5. MOVE TO GRASP APPROACH
        # -------------------------------------------------

        grasp_target = PoseStamped()

        grasp_target.header.frame_id = "world"
        grasp_target.header.stamp = detected.header.stamp

        grasp_target.pose.position.x = box_x
        grasp_target.pose.position.y = box_y
        grasp_target.pose.position.z = (
            box_top_z + GRASP_CLEARANCE
        )

        grasp_target.pose.orientation = pregrasp_target.pose.orientation

        grasp_success = plan_and_execute(
            robot,
            arm,
            grasp_target,
            "grasp approach",
        )

        if not grasp_success:
            print("\nStopping: grasp approach was not completed.")
            return

        # -------------------------------------------------
        # 6. ATTACH THE BOX IN GAZEBO
        # -------------------------------------------------

        if not attach_box(lidar_node):
            print(
                "\nStopping: Gazebo could not attach the box. "
                "The transport motion will not be attempted."
            )
            return

        box_attached = True

        # -------------------------------------------------
        # 7. MOVE THE BOX TO THE DESTINATION
        # -------------------------------------------------

        place_target = PoseStamped()

        place_target.header.frame_id = "world"
        place_target.header.stamp = detected.header.stamp

        place_target.pose.position.x = place_x
        place_target.pose.position.y = place_y

        # Use the same tool height as the grasp-approach pose.
        # This aims to put the box back at approximately its
        # original height when it reaches the destination.
        place_target.pose.position.z = grasp_target.pose.position.z

        place_target.pose.orientation = grasp_target.pose.orientation

        place_success = plan_and_execute(
            robot,
            arm,
            place_target,
            "transport box to placement location",
        )

        if not place_success:
            print(
                "\nPlacement motion did not complete. "
                "The box remains attached in Gazebo."
            )
            return

        # -------------------------------------------------
        # 8. DETACH THE BOX
        # -------------------------------------------------

        if not detach_box(lidar_node):
            print(
                "\nWARNING: Detachment failed. "
                "The box may still be attached to the robot."
            )
            return

        box_attached = False

        # -------------------------------------------------
        # 9. RETURN THE ARM HOME
        # -------------------------------------------------

        home_success = plan_and_execute_named(
            robot,
            arm,
            HOME_CONFIGURATION,
            "return robot to home",
        )

        if home_success:
            print("\nPick-and-place sequence completed.")
        else:
            print(
                "\nThe box was placed, but the return-home "
                "motion did not complete."
            )

        # -------------------------------------------------
        # 10. KEEP NODE ALIVE
        # -------------------------------------------------

        print("\nPress Ctrl+C to exit.")

        while rclpy.ok():
            rclpy.spin_once(
                lidar_node,
                timeout_sec=0.5,
            )

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    except Exception as exc:
        print(f"\nERROR: {exc}")
        raise

    finally:

        if box_attached:
            print(
                "\nWARNING: The script is exiting while the box "
                "may still be attached in Gazebo. If the simulation "
                "is still running, detach it manually or restart "
                "the simulation before the next run."
            )

        lidar_node.destroy_node()
        reader.destroy_node()

        if robot is not None:
            robot.shutdown()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
