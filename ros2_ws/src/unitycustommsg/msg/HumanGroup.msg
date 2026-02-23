# A single human group (center, diameter, and identifiers)
std_msgs/Header header      # frame_id = map (or your global frame), stamp = now
string name                 # e.g., "group_0"
string child_frame_id       # e.g., "group_0" (for TF broadcasting if you want)
geometry_msgs/Pose center   # center pose (z = 0, yaw in orientation)
float32 diameter            # minimal enclosing circle diameter in meters