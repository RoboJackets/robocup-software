float32 length
float32 width
float32 border
float32 line_width
float32 goal_width
float32 goal_depth
float32 goal_height
float32 penalty_short_dist
float32 penalty_long_dist
float32 arc_radius
float32 center_radius
float32 center_diameter
float32 goal_flat
float32 floor_length
float32 floor_width

geometry2d_msgs/Point center_point
geometry2d_msgs/Rect our_goal_zone_shape
geometry2d_msgs/Rect their_goal_zone_shape
geometry2d_msgs/Segment our_goal_segment
geometry2d_msgs/Segment their_goal_segment
geometry2d_msgs/Rect our_half
geometry2d_msgs/Rect their_half
geometry2d_msgs/Rect field_rect

# This was originally a vector, but borders have to be
# rectangular, right?
geometry2d_msgs/Line[4] field_borders
