std_msgs/Header header
string human_name              # copy from Human.name if you use it, else empty
float32 distance               # robot-to-human distance in meters (XZ plane)
string zone                    # "critical" or "personal"
geometry_msgs/Pose pose        # human pose (ROS: x->x, z->z; y=0)