**tf2_pose.py:** This script sets up a ROS2 node called `TFListener` that listens for transformations between `odom` and `base_footprint` frames, calculates the position, orientation, and velocity in Unity's coordinate system, and publishes this data in several message types.


- **TFListener:** Initializes ROS publishers for `TransformStamped`, `Twist`, and custom `TwistTransformUnity` messages.


- **tf_callback:** Executes on a timer, fetching transformations, converting to Unity coordinates, and publishing updated position and velocity.


- **quaternion_to_euler:** Converts quaternions to Euler angles(in Radians).
1

- **Transformations:** Positions and orientations are transformed using matrices (`transformation_matrix_tf` and `transformation_matrix_tf_rot`) to match Unity's coordinate system.


**Publishes:**

- robot_pose `TransformStamped`: Robot's transformed position and orientation.
- robot_velocity `Twist`: Linear and angular velocity.
- robot `TwistTransformUnity`: Combined pose and velocity message.