# Scripts

## Project Structure
### Controllers
* BedController.cs: Low-level controller for a bed model used in simulations.
* RobotController.cs:  Low-level controller for a robot model used in simulations.
### Physics and Velocity Calculations
* CalculateVelocity.cs: Contains functions to calculate and adjust velocity for physics simulations.
* GetVelocityWithRigidbody.cs: Improved velocity calculations that utilize Rigidbody for more accurate physics modeling.
### Camera and Image Processing
* ImageCapture.cs: Responsible for capturing images and improving camera functionality within the simulation.
* ImageToWorldConversion.cs: Transforms image coordinates to world coordinates for integration with ROS.
### ROS Communication
* CameraImagePublishToROS.cs: Publishes camera images from Unity to ROS.
* ROSClockPublisher.cs: Publishes simulation time to ROS, synchronizing with ROS time.
* ROSTransformTreePublisher.cs: Publishes transform data from Unity's coordinate system to ROS.
* ROSMessage.cs: Contains custom ROS message types and data structures for efficient communication.
* ROSPublisher.cs: Generic publisher for sending data from Unity to ROS.
* RobotPosePublisher.cs: Publishes robot pose information to ROS, updated with map changes and resolution fixes.
* BoundingBoxPublisher.cs: Publishes bounding box data (likely around humans/objects) to ROS for visualization or tracking.
* CameraImageToOccupancyGrid.cs: Converts camera image data into an occupancy grid format and publishes to ROS.
* RosPublisherCustom.cs: A specialized ROS publisher that handles custom message formats beyond the generic RosPublisher.

sendImageToROS.cs: Publishes raw or processed images to ROS (may overlap with CameraImagePublishToROS, but likely more direct).
### Sensors
* LaserScanSensor.cs: Simulates a Laser Scanner in Unity, publishing scan data to ROS.
### Collision Detection
* CollisionDetection.cs: Detects and handles collisions within the Unity environment, updating relevant ROS nodes.
### Visualization Tools
* PoseTrailVisualizer.cs: Visualizes the trajectory of objects to track their movement over time.
* PoseTrailVisualizer_Traj.cs: Draws reference trajectories in Unity based on ROS input data.
### Transform and Utility Scripts
* TransformExtensions.cs: Contains extensions for handling transforms within Unity.
* TransformTreeNode.cs: Represents nodes in a transform tree for hierarchical organization.
* TimeStamp.cs: Used for timestamping data in synchronization with ROS.
* mousePositionOnGame.cs: Debug tool for tracking mouse position within the Unity scene.

### Controllers / Simulation Logi

* GameManager.cs: Manages scene-level logic like spawning, setup, and global simulation state.
* FreeCam.cs: Free-movement camera controller for debugging and navigating around the Unity scene.
* 
### Human & Randomized Movement

* RandomMovement.cs: Moves entities randomly within defined constraints—simpler behavior than RandomWalker.
* RandomWalker.cs: Implements random-walk style movement with step-based changes in direction.
* SmoothRandomWalker.cs: A smoother variant of RandomWalker, reducing jittery movement for more natural paths.