# RJ Actions

Agents can take actions to interact with the world.  These are the ones I think are useful

## Go To Point

Go to a given point ignoring heading

Format:
* rj_geometry_msgs/Point target <- the point to move to
* bool avoid_ball <- should the controller treat the ball as an obstacle

## Go To Pose

Go to a given pose (including heading)

Format:
* rj_geometry_msgs/Pose target <- the pose to get to
* bool avoid_ball <- should the controller treat the ball as an obstacle

## Shoot

Shoot the ball at the goal (effectively a line kick where we drive through the ball towards the goal)

Format:
* float64 goal_location <- the position in the goal (from -1 to 1) to shoot the ball at
* float64 power <- the power to kick the ball with

## Collect

Collect the ball (basically go up to it and turn on the dribbler)

Format:

## Pass

Pass the ball to a specific robot (pivot kick)

Format:
* uint8 robot_id <- the id of the robot to pass to
* float64 power <- the power to kick the ball with

## Dribble

Move with the ball to a specified location

Format:
* rj_geometry_msgs/Pose target <- the pose to get to

## Clear

Clear the ball to a specied location (using a line chip)

Format:
* rj_geometry_msgs/Point clear_target <- the target point to cleare to
* float64 power

## Mark Robot

Stay at a given distance between an opponents robot and our goal

Format:
* uint8 their_robot_id <- the robot id of the opposing robot to mark
* float64 distance <- the distance from the opposing robot to stay


