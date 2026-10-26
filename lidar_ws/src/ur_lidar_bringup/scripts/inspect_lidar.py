#!/usr/bin/env python3

import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseStamped


class BoxDetector(Node):
    def __init__(self):
        super().__init__("box_detector")

        # Known box geometry (metres)
        self.box_length_x = 0.20
        self.box_length_y = 0.20
        self.box_height = 0.25
        self.ground_z = 0.0

        # Assumption for this first test:
        # the visible face is the negative-X face.
        self.face_tolerance_x = 0.01

        self.subscription = self.create_subscription(
            PointCloud2,
            "/lidar/points",
            self.cloud_callback,
            10,
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            "/detected_box/top_center",
            10,
        )

        self.get_logger().info(
            "Box detector started; waiting for /lidar/points"
        )

    def cloud_callback(self, msg):
        # Extract XYZ points.
        raw = list(
            point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        )

        if not raw:
            self.get_logger().warning("Point cloud contains no valid points")
            return

        # Handle structured arrays returned by sensor_msgs_py.
        points = np.asarray(raw)

        if points.dtype.names:
            points = np.column_stack(
                [points["x"], points["y"], points["z"]]
            )

        points = points.astype(np.float64)
        points = points[np.isfinite(points).all(axis=1)]

        if len(points) < 10:
            self.get_logger().warning(
                f"Too few valid points: {len(points)}"
            )
            return

        # First version: the visible face should be nearly
        # perpendicular to X, so its X coordinates are clustered.
        #
        # Use a narrow band at the minimum X value to select
        # the near face. This works for the current test scene,
        # where the cloud predominantly contains the box face.
        x_face = np.min(points[:, 0])

        face_points = points[
            points[:, 0] <= x_face + self.face_tolerance_x
        ]

        if len(face_points) < 10:
            self.get_logger().warning(
                "Not enough points on the candidate face"
            )
            return

        # Robust face bounds: percentiles reduce sensitivity
        # to isolated noisy points.
        y_low, y_high = np.percentile(
            face_points[:, 1], [2, 98]
        )

        z_low, z_high = np.percentile(
            face_points[:, 2], [2, 98]
        )

        face_width_y = y_high - y_low
        face_height_z = z_high - z_low

        # Validate the detected face against known dimensions.
        # Allow some tolerance for incomplete LiDAR coverage.
        if not (0.12 <= face_width_y <= 0.24):
            self.get_logger().warning(
                f"Candidate face width looks wrong: "
                f"{face_width_y:.3f} m"
            )
            return

        self.get_logger().info(
            f"Candidate face points: {len(face_points)}"
        )
        self.get_logger().info(
            f"Face X estimate: {x_face:.4f} m"
        )
        self.get_logger().info(
            f"Face Y extent: [{y_low:.4f}, {y_high:.4f}] m"
        )
        self.get_logger().info(
            f"Observed Z extent: [{z_low:.4f}, {z_high:.4f}] m"
        )
        self.get_logger().info(
            f"Observed face height: {face_height_z:.4f} m"
        )

        # Infer the box center from the negative-X face.
        center_x = x_face + self.box_length_x / 2.0
        center_y = (y_low + y_high) / 2.0

        # The box rests flat on the known ground plane.
        top_z = self.ground_z + self.box_height

        self.get_logger().info(
            f"Inferred top center: "
            f"x={center_x:.4f}, "
            f"y={center_y:.4f}, "
            f"z={top_z:.4f}"
        )

        # Publish the estimated top-surface center.
        #
        # IMPORTANT: this first version assumes that the
        # LiDAR coordinates coincide with world coordinates.
        # We will verify that assumption before using this
        # pose to command the robot.
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "world"

        pose.pose.position.x = float(center_x)
        pose.pose.position.y = float(center_y)
        pose.pose.position.z = float(top_z)

        # Identity orientation: box is assumed axis-aligned.
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()