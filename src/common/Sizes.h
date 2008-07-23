#ifndef SIZES_H
#define SIZES_H

//all distances in meters

/** Diameter of the ball */
#define BALL_DIAM		(.043f)

#define BALL_RADIUS		(BALL_DIAM/2.0f)

/** Robot max diameter */
#define ROBOT_DIAM		(.180f)

#define ROBOT_RADIUS	(ROBOT_DIAM/2.0f)

/** Diameter of the team color marker */
#define MARKER_DIAM		(.050f)

/** The depth of the goal from the line to the wall */
#define GOAL_DEPTH		(.180f)
/** Thickness of the goal walls */
#define GOAL_THICKNESS	(.020f)
/** Width of the goal (distance between goal walls */
#define GOAL_WIDTH		(.700f)
/** Distance from the center of the goal to one wall (for convenience) */
#define GOAL_LIMIT      (GOAL_WIDTH / 2)

/** Width of the field line markings */
#define LINE_WIDTH		(.010f)
/** Radius of the goal arcs */
#define ARC_RADIUS		(.5f)

/** Field length from goal to goal */
#define FIELD_LENGTH	(6.1f) //(5.000f)
/** Field width */
#define FIELD_WIDTH		(4.2f) //(3.500f)

/** Space around the field outline */
#define FIELD_DEADSPACE (.250f)

/** Length of the entire floor (goal to goal direction) */
#define FLOOR_LENGTH	(FIELD_LENGTH + 2.0*FIELD_DEADSPACE)
/** Width of the floor (perp to length) */
#define FLOOR_WIDTH		(FIELD_WIDTH + 2.0*FIELD_DEADSPACE)

/** Disstance of the penalty marker from the goal line */
#define PENALTY_DIST	(.450f)
/** Diameter of the penalty marker */
#define PENALTY_DIAM	(.010f)

/** height per one unit length */
#define FEILD_ASPECT	(FLOOR_WIDTH/FLOOR_LENGTH)

/** radius of ball avoidance zone during stoppage */
#define BALL_AVOID		(.5f)

#endif /* SIZES_H */
