import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from tf_transformations import euler_from_quaternion
from unitycustommsg.msg import Point2DArray, TwistTransformUnity

# ---------------- CONFIG ---------------- #


def load_config():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.normpath(
        os.path.join(
            base_dir,
            "../../../../../../../floor-segmentation/json/config.json",
        )
    )

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Config not found: {config_path}")

    with open(config_path, "r") as f:
        return json.load(f)


# ---------------- NODE ---------------- #


class PathEvaluationNode(Node):
    def __init__(self):
        super().__init__("path_evaluation_node")

        # ---- State flags ----
        self.evaluated = False

        # ---- Data storage ----
        self.robot_position = None
        self.robot_yaw = None
        self.robot_positions = []
        self.global_trajectory = []
        self.path_start_time = None

        # ---- Load config ----
        config = load_config()

        self.start_point = [
            config["robot"]["start"]["position"]["x"],
            config["robot"]["start"]["position"]["y"],
        ]

        self.goal_point = [
            config["pathPlanning"]["goal"]["position"]["x"],
            config["pathPlanning"]["goal"]["position"]["y"],
        ]

        self.destination_threshold = config["pathPlanning"]["goal"][
            "destination_threshold"
        ]

        self.expected_heading_deg = math.degrees(
            config["pathPlanning"]["goal"]["theta"]
        )

        # ---- QoS (latched results) ----
        qos_latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # ---- Publishers ----
        self.pub_reached_goal = self.create_publisher(
            Bool, "/path_eval/reached_goal", qos_latched
        )
        self.pub_completion_time = self.create_publisher(
            Float32, "/path_eval/completion_time", qos_latched
        )
        self.pub_orientation_error = self.create_publisher(
            Float32, "/path_eval/orientation_error_deg", qos_latched
        )
        self.pub_distance_error = self.create_publisher(
            Float32, "/path_eval/distance_error", qos_latched
        )
        self.pub_global_len = self.create_publisher(
            Float32, "/path_eval/global_path_length", qos_latched
        )
        self.pub_robot_len = self.create_publisher(
            Float32, "/path_eval/robot_travel_distance", qos_latched
        )
        self.pub_efficiency = self.create_publisher(
            Float32, "/path_eval/efficiency", qos_latched
        )
        self.pub_summary = self.create_publisher(
            String, "/path_eval/summary", qos_latched
        )

        # ---- Subscribers ----
        self.create_subscription(TwistTransformUnity, "/robot", self.robot_callback, 10)
        self.create_subscription(
            Point2DArray, "/global_trajectory", self.global_trajectory_callback, 10
        )
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # ---- cmd_vel stop detection ----
        self.last_cmd_time = None
        self.cmd_seen_once = False
        self.cmd_timeout_s = 1.5

        self.create_timer(0.1, self.check_cmd_vel_timeout)

        self.get_logger().info("PathEvaluationNode initialized.")

    # ---------------- Callbacks ---------------- #

    def robot_callback(self, msg: TwistTransformUnity):
        t = msg.transform.transform.translation
        r = msg.transform.transform.rotation

        self.robot_position = (t.x, t.y)

        _, _, yaw = euler_from_quaternion([r.x, r.y, r.z, r.w])
        self.robot_yaw = yaw

        self.robot_positions.append(self.robot_position)

        if self.path_start_time is None:
            self.path_start_time = time.time()

    def global_trajectory_callback(self, msg: Point2DArray):
        self.global_trajectory = [(p.x, p.y) for p in msg.points]

    def cmd_vel_callback(self, _msg: Twist):
        self.last_cmd_time = time.time()
        self.cmd_seen_once = True

        if self.path_start_time is None:
            self.path_start_time = time.time()

    def check_cmd_vel_timeout(self):
        if self.evaluated:
            return

        if (
            not self.cmd_seen_once
            or self.last_cmd_time is None
            or self.robot_position is None
            or len(self.global_trajectory) < 2
        ):
            return

        if (time.time() - self.last_cmd_time) >= self.cmd_timeout_s:
            self.evaluate_path()

    # ---------------- Metrics ---------------- #

    def compute_path_length(self, path):
        if len(path) < 2:
            return 0.0

        return sum(
            math.hypot(
                path[i][0] - path[i - 1][0],
                path[i][1] - path[i - 1][1],
            )
            for i in range(1, len(path))
        )

    def compute_orientation_error(self):
        if self.robot_yaw is None:
            return None

        robot_deg = math.degrees(self.robot_yaw) % 360
        expected = self.expected_heading_deg % 360
        diff = abs(robot_deg - expected)
        return min(diff, 360 - diff)

    # ---------------- Evaluation ---------------- #

    def evaluate_path(self):
        if self.evaluated:
            return
        self.evaluated = True

        completion_time = (
            time.time() - self.path_start_time if self.path_start_time else 0.0
        )

        global_len = self.compute_path_length(self.global_trajectory)
        robot_len = self.compute_path_length(self.robot_positions)

        efficiency = robot_len / global_len if global_len > 0 else None

        distance_error = math.hypot(
            self.robot_position[0] - self.goal_point[0],
            self.robot_position[1] - self.goal_point[1],
        )

        orientation_error = self.compute_orientation_error()

        # ---- Publish ----
        self.pub_reached_goal.publish(Bool(data=True))
        self.pub_completion_time.publish(Float32(data=completion_time))
        self.pub_distance_error.publish(Float32(data=distance_error))
        self.pub_global_len.publish(Float32(data=global_len))
        self.pub_robot_len.publish(Float32(data=robot_len))

        if orientation_error is not None:
            self.pub_orientation_error.publish(Float32(data=orientation_error))

        if efficiency is not None:
            self.pub_efficiency.publish(Float32(data=efficiency))

        summary = (
            f"completion_time_s={completion_time:.3f}, "
            f"orientation_error_deg={orientation_error if orientation_error is not None else float('nan'):.3f}, "
            f"distance_error_m={distance_error:.3f}, "
            f"global_len_m={global_len:.3f}, "
            f"robot_len_m={robot_len:.3f}, "
            f"efficiency={efficiency if efficiency is not None else float('nan'):.4f}"
        )

        self.pub_summary.publish(String(data=summary))
        self.get_logger().info(summary)


# ---------------- MAIN ---------------- #


def main(args=None):
    rclpy.init(args=args)
    node = PathEvaluationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
