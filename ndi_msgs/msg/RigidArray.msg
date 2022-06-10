# This message holds a collection of 3d poses of rigid bodies, plus optional additional
# information about each body from the mocap system.

# Time of sensor data acquisition, coordinate frame ID.
std_msgs/Header header

#Array of point IDs.
int32[] ids

# Array of 3d poses. Each Pose should be interpreted as a 3d pose
# in the frame given in the header.
geometry_msgs/Pose[] poses
