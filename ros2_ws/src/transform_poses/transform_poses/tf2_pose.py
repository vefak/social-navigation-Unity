#!/usr/bin/env python3

import math
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from unitycustommsg.msg import TwistTransformUnity


class TFListener(Node):
    def __init__(self):
        super().__init__("tf_listener")
        self.get_logger().info("Started listener node!")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.robot_pose_publisher = self.create_publisher(
            TransformStamped, "robot_pose", 10
        )
        self.velocity_publisher = self.create_publisher(Twist, "robot_velocity", 10)
        self.robot_publisher = self.create_publisher(TwistTransformUnity, "robot", 10)
        self.create_timer(0.01, self.tf_callback)

        self.transformation_matrix_tf = np.array([[0, -1, 0], [0, 0, 1], [1, 0, 0]])
        self.transformation_matrix_tf_rot = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])

        # Variables to store previous position and time
        self.prev_position = None
        self.prev_orientation = None
        self.prev_time = None

    def quaternion_to_euler(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return np.array([roll, pitch, yaw])

    def tf_callback(self):
        try:
            # transform = self.tf_buffer.lookup_transform(
            #    "odom", "base_footprint", rclpy.time.Time()
            # )
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", rclpy.time.Time()
            )
            current_time = time.time()

            # Get ROS Points Translation
            ros_point = np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ]
            )
            unity_point = np.dot(self.transformation_matrix_tf, ros_point)

            # Get current orientation in Euler angles
            ros_rotation = self.quaternion_to_euler(transform.transform.rotation)
            unity_rotation = np.dot(self.transformation_matrix_tf_rot, ros_rotation)

            # Initialize velocity_msg with default values
            velocity_msg = Twist()

            # Calculate velocity if we have a previous position and time
            if self.prev_position is not None and self.prev_time is not None:
                dt = current_time - self.prev_time
                if dt > 0:  # To avoid division by zero
                    linear_velocity = (unity_point - self.prev_position) / dt
                    angular_velocity = (unity_rotation - self.prev_orientation) / dt

                    # Set a threshold to filter out very small values
                    threshold = 1e-3
                    linear_velocity = np.where(
                        np.abs(linear_velocity) < threshold, 0, linear_velocity
                    )
                    angular_velocity = np.where(
                        np.abs(angular_velocity) < threshold, 0, angular_velocity
                    )
                    # Round velocities to 3 decimal places
                    linear_velocity = np.round(linear_velocity, 4)
                    angular_velocity = np.round(angular_velocity, 4)

                    # Populate the velocity message with calculated values
                    velocity_msg.linear.x = linear_velocity[0]
                    velocity_msg.linear.y = linear_velocity[1]
                    velocity_msg.linear.z = linear_velocity[2]
                    velocity_msg.angular.x = angular_velocity[0]
                    velocity_msg.angular.y = angular_velocity[1]
                    velocity_msg.angular.z = angular_velocity[2]
                    self.velocity_publisher.publish(velocity_msg)

            # Update previous position, orientation, and time
            self.prev_position = unity_point
            self.prev_orientation = unity_rotation
            self.prev_time = current_time

            # Round translation and rotation values to 3 decimal places
            unity_point = np.round(unity_point, 4)
            unity_rotation = np.round(unity_rotation, 4)

            # Publish robot pose message
            robot_pose_msg = TransformStamped()
            robot_pose_msg.header.frame_id = "base_footprint"
            robot_pose_msg.child_frame_id = "robot"
            robot_pose_msg.transform.translation.x = unity_point[0]
            robot_pose_msg.transform.translation.y = unity_point[1]
            robot_pose_msg.transform.translation.z = unity_point[2]
            robot_pose_msg.transform.rotation.x = unity_rotation[0]
            robot_pose_msg.transform.rotation.y = unity_rotation[1]
            robot_pose_msg.transform.rotation.z = unity_rotation[2]
            robot_pose_msg.transform.rotation.w = transform.transform.rotation.w
            self.robot_pose_publisher.publish(robot_pose_msg)

            # Publish the custom TwistTransformUnity message
            twist_transform_msg = TwistTransformUnity()
            twist_transform_msg.twist = velocity_msg
            twist_transform_msg.transform = robot_pose_msg
            self.robot_publisher.publish(twist_transform_msg)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().info("TF Exception: " + str(e))


def main(args=None):
    rclpy.init(args=args)
    tf_listener = TFListener()
    rclpy.spin(tf_listener)
    tf_listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
