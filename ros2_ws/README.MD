
# ROS2 Workspace: ROS2 WORKSPACE

This is a ROS2 workspace containing multiple packages related to Project. Each package provides specific functionality, ranging from robot control to image processing and database management.
## Table of Contents

- [ROS2 Workspace: ROS2 WORKSPACE](#ros2-workspace-ros2-workspace)
  - [Table of Contents](#table-of-contents)
  - [Workspace Setup](#workspace-setup)
  - [ROS Packages Overview](#ros-packages-overview)
  - [Sources](#sources)


## Workspace Setup

1. Install necessary dependencies:
   ```bash
   cd ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
   - Note: You may encounter warnings during the build process from the following packages:
     - neo4j
     - pybind11
     - eigen3
2. Install pybind11
    ```bash
    pip install pybind11
    ```
3. Install Neo4j

- Refer to [Neo4j page](neo4j/README.MD)
  
4. Install eigen3
    ```bash
    sudo apt install libeigen3-dev
    ```
5. Install nav2
   ```bash
   sudo apt install ros-humble-nav*
   ```
6. Build & Source your ROS2 environment (Or you can add it to your .bashrc file)
   ```bash
   colcon build
   source /opt/ros/humble/setup.bash
   ```
7. Put below command in .bashrc file (If you have multiple ws, not recommend)
  ```bash
      source /opt/ros/humble/setup.bash
  ```
## ROS Packages Overview
Detailed package descriptions are included within that folder.
  - [database_updater](src/database_updater)
  - [floor_segmentation](src/floor_segmentation)
  - [image_subscriber](src/image_subscriber)
  - [ROS-TCP-Endpoint](src/ROS-TCP-Endpoint)
  - [stanley_controller](src/stanley_controller)
  - [unitycustommsg](src/unitycustommsg)
  - [common_interfaces](src/common_interfaces)
  - [costmap_plugin](src/costmap_plugin)
  - [evaluations](src/evaluations)
  - [project_main](src/project_main)
  - [transform_poses](src/transform_poses/)
## Sources

[1] [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)

[2] [Unity Set .msg & .srv files](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md)

[3] [Neo4j](https://neo4j.com/)

[4] [URDF Importer Unity](https://github.com/Unity-Technologies/URDF-Importer)

[5] [Custom ROS2 msg and srv files](https://docs.ros.org/en/crystal/Tutorials/Custom-ROS2-Interfaces.html)

[6] [Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing](https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf)

[7] [Nav2 - Open Navigation](https://docs.nav2.org/)