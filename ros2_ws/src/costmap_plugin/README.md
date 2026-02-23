
## costmap_plugin
Custom ROS 2 costmap plugin for integrating dynamic obstacle positions (from Unity) into the Nav2 local costmap.  
This package allows robots to navigate in human-populated environments by treating humans as dynamic obstacles.


### Features

- **Dynamic Human Layer Plugin**
  - Implements a costmap layer that injects human obstacle data into Nav2.
  - Receives human positions (e.g., from Unity via ROS2 topics).
  - Updates the local costmap in real-time.

- **Map Integration**
  - Contains a Unity-generated floor plan (`floor_plan_unity.pgm` / `.png`) and ROS 2 `map.yaml`.
  - Can be used to load an environment map with `map_server`.

- **Nav2 Configurations**
  - `nav2_with_human_layer.yaml`: Includes plugin setup for Nav2 stack.
  - `dwb.yaml`: Configures DWB controller for local navigation.


### Plugin Registration

The plugin is registered in plugins.xml:
```
<library path="lib/libdynamic_human_layer">
<class name="costmap_plugin::DynamicHumanLayer"
        type="costmap_plugin::DynamicHumanLayer"
        base_class_type="nav2_costmap_2d::Layer"/>
</library>
```

### Dependencies

* nav2_costmap_2d
* nav2_core
* geometry_msgs
* rclcpp
* 
### Notes

* Human topics must provide transforms or pose data in ROS 2.

### YAML Configuration

#### Map Server

```
map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: /.../map/map.yaml
    frame_id: "map"
    subscribe_to_updates: true
```

* Loads the static map (map.yaml).
* Uses simulation time.
* Publishes in map frame.
* Subscribes to updates so the map can be modified dynamically.

#### Controller Server

```
controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
```

* Runs the MPPI controller (nav2_mppi_controller::MPPIController)
* Executes at 20 Hz.
* Configured for Ackermann kinematics (car-like robots).
* Sampling parameters (time_steps, batch_size, temperature, etc.) control rollout-based trajectory optimization.
* Velocity/acceleration limits enforce physical feasibility.
* Critics (GoalCritic, PathAlignCritic, ObstaclesCritic, etc.) score trajectories:
  * ConstraintCritic / CostCritic: Collision and footprint costs.
  * GoalCritic / GoalAngleCritic: Drive towards goal and align heading.
  * PathAlignCritic / PathFollowCritic / PathAngleCritic: Stay aligned to path.
  * PreferForwardCritic: Bias against reversing.
  * ObstaclesCritic: Strong repulsion from humans/obstacles.
* Goal checker ensures final tolerance in position (0.25 m) and orientation (0.35 rad).


#### Local Costmap
```
local_costmap:
  ros__parameters:
    rolling_window: true
    width: 15
    height: 15
    resolution: 0.0375
    update_frequency: 10.0
    publish_frequency: 5.0
    plugins: [static_layer, obstacle_layer, dynamic_human_layer, inflation_layer]
```
* Defines a rolling 15x15 m window around the robot.
* High resolution (0.0375 m per cell).
* Updates quickly (10 Hz).
* Layers:
  * Static Layer: Background map.
  * Obstacle Layer: Standard sensor-based obstacles.
  * Dynamic Human Layer: Custom plugin (unitycustom_layers::DynamicHumanLayer), adding human positions with human_radius and core_radius.
  * Inflation Layer: Expands costs outward so paths keep clearance (0.6 m).

#### Global Costmap
```
global_costmap:
  ros__parameters:
    width: 20
    height: 20
    resolution: 0.05
    update_frequency: 1.0
    plugins: [static_layer, obstacle_layer, inflation_layer]
```
* Provides a global navigation map.
* 20x20 m area, lower resolution (0.05 m).
* Updated at slower rate (1 Hz).
* Layers: static + obstacle + inflation (but no human layer here).