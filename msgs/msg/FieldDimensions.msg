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

Point center_point
Rect our_goal_zone_shape
Rect their_goal_zone_shape
Segment our_goal_segment
Segment their_goal_segment
Rect our_half
Rect their_half
Rect field_rect

# This was originally a vector, but borders have to be
# rectangular, right?
Line[4] field_borders
