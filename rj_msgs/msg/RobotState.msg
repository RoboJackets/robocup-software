# This message contains data that comes from the vision system
# including position data and which camera this robot was seen by and
# what time it was last seen.
# TODO(?): Revisit the "which camera" part. It seems to be missing in WorldState.hpp.

builtin_interfaces/Time stamp
rj_geometry_msgs/Pose pose
rj_geometry_msgs/Twist velocity
bool visible
