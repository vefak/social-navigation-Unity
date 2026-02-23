## evaluations

ROS 2 Python package for **trajectory and navigation performance evaluation** in the project.  
It computes metrics (per step and aggregate), supports multi-objective scoring, and can stream results to logs or a database.


---
#### evaluations.py | PathEvaluationNode (runs online during a nav run)

subscribe to Unity/ROS2 topics, watch the robot until it reaches the goal, then compute a handful of path metrics (distances, orientation error, efficiency).

**Subscriptions**

* /robot (unitycustommsg/…/TwistTransformUnity): robot pose & twist.
* /global_trajectory (Point2DArray): the nav path (global).
* /reference_trajectory (Point2DArray): external reference path; code tries to accumulate non-overlapping segments across updates.

**Key state**

* robot_positions: breadcrumb list of (x, y) as the robot moves.
* global_trajectory: list of (x, y) from /global_trajectory.
* stored_points: accumulated reference-trajectory segments across messages.
* goal_point: hardcoded [0.75, 0.0] (used instead of end of global path).


When distance to goal_point ≤ destination_threshold (2m), call evaluate_path().

#### Metrics printed:

* compute_orientation_error() — angle gap (deg) to expected_heading_deg.
* compute_distance_error() — distance robot→last global path point.
* compute_global_trajectory_length() — sum of segment lengths.
* compute_reference_trajectory_length() — sum over stored_points.
* compute_robot_travel() — sum of traveled segments from breadcrumbs.
* calculate_efficiency_between_distances() — ratio travel/global.


#### mdo_calculator.py — ObstacleDistanceCalculator (actually: distance-to-obstacle node)

It computes nearest-obstacle distance from an occupancy grid using a distance transform and TF to the robot

**Subscriptions**

* /occupancy_grid_map (nav_msgs/OccupancyGrid)
* /tf
