# project_main/launch/project_main__launch.py

import json
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_recordings_path():
    workspace_root = os.path.abspath(os.path.join(os.getcwd(), ".."))
    package_src_dir = os.path.join(workspace_root, "recordings")
    return package_src_dir


def generate_launch_description():
    package_name = "project_main"
    package_dir = get_package_share_directory(package_name)

    # ---- Args ----
    record_bag = DeclareLaunchArgument(
        "record_bag", default_value="false", description="Whether to record a ROS2 bag"
    )
    test_name = DeclareLaunchArgument(
        "test_name", default_value="demo", description="Name of test/Use Case"
    )

    # Nav2 args
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="./src/costmap_plugin/map/map.yaml",
        description="Full path to map yaml",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value="./src/costmap_plugin/nav2_with_human_layer.yaml",
        description="Full path to Nav2 params yaml",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo/Unity) clock",
    )

    # RViz args
    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="Launch RViz2 with Nav2 default config",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("project_main"),
                "rviz",
                "project.rviz",
            ]
        ),
        description="Full path to RViz config",
    )

    # ---- Bag output folder ----
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    recordings_path = get_recordings_path()
    output_folder = PathJoinSubstitution(
        [
            TextSubstitution(text=recordings_path),
            LaunchConfiguration("test_name"),
            TextSubstitution(text=f"{timestamp}"),
        ]
    )

    record_bag_action = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-a", "-s", "mcap", "-o", output_folder],
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )

    # ---- Nodes ----
    nodes = [
        # TCP endpoint (Unity bridge)
        Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            name="tcp_endpoint_node",
            output="log",
            parameters=[{"ROS_IP": "172.20.1.223"}],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("show_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        # Static TF: map -> odom  (start early so Nav2 has TFs)
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="map_to_odom_static_broadcaster",
                    output="log",
                    # ros2 run tf2_ros static_transform_publisher x y z qx qy qz qw frame_id child_frame_id
                    arguments=[
                        "36",
                        "13.5",
                        "0",
                        "0",
                        "0",
                        "0.707",
                        "0.707",
                        "map",
                        "odom",
                    ],
                )
            ],
        ),
        # tf2 pose node
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="transform_poses",
                    executable="tf2_pose",
                    name="tf2_pose_node",
                    output="log",
                )
            ],
        ),
        # groups pose node
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="transform_poses",
                    executable="groups",
                    name="groups_node",
                    output="log",
                )
            ],
        ),
        # Neo4j updater
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="database_updater",
                    executable="neo4j_node",
                    name="neo4j_node",
                    output="log",
                )
            ],
        ),
        # Human aggregator
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="human_aggregator_node",
                    executable="human_aggregator",
                    name="human_aggregator",
                    output="log",
                )
            ],
        ),
        # Nav2 bringup (replaces Stanley)
        # Equivalent to:
        # ros2 launch nav2_bringup bringup_launch.py map:=... params_file:=... use_sim_time:=true
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("nav2_bringup"),
                                "launch",
                                "bringup_launch.py",
                            ]
                        )
                    ),
                    launch_arguments={
                        "map": LaunchConfiguration("map"),
                        "params_file": LaunchConfiguration("params_file"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        # optional but usually helpful:
                        "autostart": "true",
                    }.items(),
                )
            ],
        ),
        # local planner (keep if you still need it)
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package="floor_segmentation",
                    executable="local_planner",
                    name="local_planner_node",
                    output="log",
                )
            ],
        ),
    ]

    return LaunchDescription(
        [
            record_bag,
            test_name,
            map_arg,
            params_file_arg,
            use_sim_time_arg,
            record_bag_action,
            show_rviz_arg,
            rviz_config_arg,
            *nodes,
        ]
    )
