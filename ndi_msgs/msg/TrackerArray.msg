# This message holds a collection of 3d tracker points, plus optional additional
# information about each point from the mocap system.

# Time of sensor data acquisition, coordinate frame ID.
std_msgs/Header header

#Array of point IDs.
int32[] ids

# Array of 3d points. Each Point should be interpreted as a 3d point
# in the frame given in the header.
geometry_msgs/Point32[] points

#Array of estimation residuals
float32[] residuals
