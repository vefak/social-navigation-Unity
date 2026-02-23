## stanley_controller
This C++ ROS2 node implements a Stanley Controller for path tracking of a mobile robot. The controller subscribes to a topic for robot position updates, processes trajectory waypoints, and publishes velocity commands to keep the robot on track.

### Key Features
* **Stanley Control:** Calculates control commands based on the robot's * lateral and heading errors relative to a reference trajectory.
* **Trajectory Discretization:** Reads or subscribes to trajectory waypoints and discretizes them.
* **Dynamic Direction Control:** Switches between forward and reverse based on path alignment and obstacles.
  
### Dependencies
* ROS2 Humble
* Eigen for linear algebra operations
* geometry_msgs for TransformStamped and Twist messages
* Custom Message Type: `unitycustommsg::msg::Point2DArray` for trajectory points

### Installation of Eigen
```BASH
sudo apt install libeigen3-dev
```

### Key Classes and Functions
#### Parameters:

* Initializes physical parameters for the robot and tuning parameters for the Stanley controller.
#### StanleyController:

* **pose_callback:** Receives robot position updates and calculates the control errors.
* **trajectory_callback:** Receives or loads waypoints and discretize them for smooth following.
* **publish_velocity_command:** Publishes calculated velocity commands to keep the robot on the path.
* **control:** Implements the Stanley controller logic.
* **controlError:** Calculates control errors based on the current robot position and target trajectory point.
#### Helper Functions:
* **discretize_points:** Resamples trajectory points based on the specified sampling rate.
* **compute_orientation_usingCardinalSplines** & **add_orientation_rad_to_points_cubic:** Computes orientations for smooth path-following.

#### Topics:

* **Subscribed:**

  * `/robot_pose`: Current robot position and orientation.
  * `/reference_trajectory`: Waypoints defining the target path.
* **Published:**
  * `/cmd_vel`: Linear and angular velocity commands for controlling the robot.

