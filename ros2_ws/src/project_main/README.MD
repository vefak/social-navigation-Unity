
#### One-button bring-up for the stack:

- starts the Unity TCP bridge
- brings up RViz (optional)
- publishes a fixed map→odom transform
- starts helper nodes (TF pose, Neo4j updater, human aggregator)
- launches Nav2 bringup with your map + params
- (optional) records a rosbag to a timestamped folder
- (optional) run time eval
- 
All of that is orchestrated with small time-staggered TimerActions to keep startup clean.

Launch files

### `launch/project_main_launch.py`


* `record_bag` (bool) – record all topics to mcap if true

* `test_name` (str) – folder label under recordings/

* `map` (path) – map YAML (defaults to your costmap_plugin/map/map.yaml)

* `params_file` (path) – Nav2 params (defaults to nav2_with_human_layer.yaml)

* `use_sim_time` (bool) – default true

* `show_rviz` (bool) – default true

* `rviz_config` (path) – default to project.rviz

#### Sequence (with delays):

1. ros_tcp_endpoint (default_server_endpoint) – Unity bridge; sets ROS_IP

2. RViz (if show_rviz) with your config

3. static TF map→odom (x=36, y=13.5, z=0; qz=0.707, qw=0.707)

4. transform_poses/tf2_pose helper

5. database_updater/neo4j_node

6. human_aggregator_node

7. nav2_bringup/bringup_launch.py with map, params_file, use_sim_time, autostart:=true
8. rosbag record


#### Important knobs & gotchas

* **ROS_IP for Unity bridge:** The launcher passes parameters=[{"ROS_IP": "172.20.1.211"}] to ros_tcp_endpoint. Change this to the ROS machine’s IP visible to Unity, or move it to a param file if your IP changes often.

* **Static TF map→odom:** The transform (36, 13.5, 0 and yaw ≈ 90°) hard-aligns Unity and ROS frames for your scene. If your world origin changes, update these numbers. Command reflects `static_transform_publisher x y z qx qy qz qw frame_id child`

* **Startup staggering:** The small TimerAction delays (1s, 2s, …) ensure TFs and the bridge exist before Nav2 starts—keeps AMCL/Costmaps happy.

* **Recording path:** Bags go to `../recordings/<test_name>/<timestamp>/` relative to the workspace (helper get_recordings_path() computes this). Make sure the folder exists or the parent is writable.

* **RViz config:** Default is `project_main/rviz/project.rviz`. Override with `rviz_config:=...` if you have scenario-specific layouts.

