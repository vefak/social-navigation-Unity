import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start_record_and_play(context, *args, **kwargs):
    """
    Starts recording, then plays the input bag, then stops recording automatically.
    We do this in ONE bash process so we can reliably stop the recorder when play ends.
    """
    bag_in = LaunchConfiguration("bag_in").perform(context)
    bag_out = LaunchConfiguration("bag_out").perform(context)
    record_all = LaunchConfiguration("record_all").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )
    play_rate = LaunchConfiguration("play_rate").perform(context)
    play_delay_sec = LaunchConfiguration("play_delay_sec").perform(context)

    # Topics that contain your "results"
    result_topics = [
        "/clock",
        "/nearest_obstacle_distance",
        "/nearest_obstacle_marker",
        "/personal_space/min_distance",
        "/personal_space/personal_breach",
        "/personal_space/critical_breach",
        "/personal_space/nearest_human",
        "/path_eval/reached_goal",
        "/path_eval/completion_time",
        "/path_eval/orientation_error_deg",
        "/path_eval/distance_error",
        "/path_eval/global_path_length",
        "/path_eval/robot_travel_distance",
        "/path_eval/efficiency",
        "/path_eval/summary",
    ]

    if record_all:
        record_cmd = f'ros2 bag record -o "{bag_out}" --storage mcap -a'
    else:
        topics_str = " ".join([f'"{t}"' for t in result_topics])
        record_cmd = f'ros2 bag record -o "{bag_out}" --storage mcap {topics_str}'

    # --clock publishes /clock; --rate optional; add any filters you want
    play_cmd = f'ros2 bag play "{bag_in}" --clock --rate {play_rate}'

    bash = f"""
set -e
echo "[eval launch] Recording to: {bag_out}"
{record_cmd} &
REC_PID=$!
echo "[eval launch] Recorder PID: $REC_PID"

sleep {play_delay_sec}

echo "[eval launch] Playing bag: {bag_in}"
{play_cmd}

echo "[eval launch] Playback finished. Stopping recorder..."
sleep 7.0
kill -INT $REC_PID
wait $REC_PID || true
echo "[eval launch] Done. Output bag: {bag_out}"
"""

    from launch.actions import ExecuteProcess

    return [ExecuteProcess(cmd=["bash", "-lc", bash], output="screen")]


def generate_launch_description():
    # --- Launch args ---
    bag_in = DeclareLaunchArgument(
        "bag_in",
        description="Input bag path (folder for sqlite3 bag, or .mcap file).",
    )
    bag_out = DeclareLaunchArgument(
        "bag_out",
        default_value="eval_results",
        description="Output bag name/folder for recorded results.",
    )
    record_all = DeclareLaunchArgument(
        "record_all",
        default_value="false",
        description="If true: record all topics (-a). If false: record only result topics.",
    )
    play_rate = DeclareLaunchArgument(
        "play_rate",
        default_value="1.0",
        description="Playback rate for ros2 bag play.",
    )
    play_delay_sec = DeclareLaunchArgument(
        "play_delay_sec",
        default_value="1.0",
        description="Delay before starting playback (gives nodes/record time to start).",
    )
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulated time from /clock (recommended when using rosbag play --clock).",
    )

    # --- Your nodes ---
    path_eval_node = Node(
        package="evaluations",
        executable="evaluations",
        name="path_evaluation_node",
        output="screen",
        # parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        parameters=[{"use_sim_time": False}],
    )

    mdo_node = Node(
        package="evaluations",
        executable="mdo_calculator",
        name="obstacle_distance_calculator",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            # optional overrides:
            # {"debug_visualization": False},
            # {"timer_period_sec": 0.5},
        ],
    )

    personal_space_node = Node(
        package="evaluations",
        executable="personal",
        name="personal_space_monitor",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            # optional overrides:
            # {"humans_topic": "/all_humans"},
            # {"personal_radius": 2.5},
            # {"critical_radius": 1.5},
            # {"humans_frame": "map"},
        ],
    )

    # Start nodes first, then record+play controller
    controller = OpaqueFunction(function=_start_record_and_play)

    return LaunchDescription(
        [
            bag_in,
            bag_out,
            record_all,
            play_rate,
            play_delay_sec,
            use_sim_time,
            path_eval_node,
            mdo_node,
            personal_space_node,
            # small delay is already in controller; this just ensures nodes start first
            TimerAction(period=0.2, actions=[controller]),
        ]
    )
