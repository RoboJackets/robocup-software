# Plays, Tactics, and Skills Summary
This document will highlight all the plays, tactics, and skills we use in our higher level play system and give a short description of the action it will take as well as a rule of thumb for use. If you want more specific information about individual plays, the next place to look is in the code itself.

## Plays
Adaptive Formation and Basic 122 are the two main plays we run at competition. Adaptive is the more advanced one, but it has not been tested as much. Additionally, it requires passing to be possible on the physical robots. Basic 122 is much simpler, but will have more trouble scoring.

| Offensive Plays    | Description |
| ------------------ | ----------- |
| Adaptive Formation | Repeatedly finds the best pass, until it has a good shot on goal |
| Basic 122          | One robot will always try to get the ball and shoot while the others defend |
| Two Side Attack    | Places two static receiver robots in the middle of the field and decides whether to pass to them or shoot |

## Tactics
For higher level plays, only Coordinated Pass and Defense are really used.

| Tactics                | Description | Use When ... |
| ---------------------- | ----------- | ------------ |
| Advance Zone Midfielder| Moves one robot to the best position to recieve the ball, and moves another robot to the best position to recieve the ball from that position. | Use it when need to move robots on a downfield attack at goal. |
| Coordinated Pass       | Moves one robot to the ball and the other to the receive point. When they both are ready, they will pass | Use when you want to pass the ball from one robot to another |
| Defense                | Basic 2 robot defense that stays on the goalie box. Clears the ball when it comes close. | Use in most cases unless you are writing your own defense |
| Defense Old            | Same as Defense, but significantly harder to read code-wise | Don't use |
| Defensive Forward      | Commands 2 of the 3 offensive robots to block while the 3rd tries to take control of the ball | Use when you're trying to capture the ball and have nothing for the other robots to do |
| Forward Pass           | Basically the same as Coordinated Pass, but ball capture is slightly different | Don't use since the new ball capture needs more testing / tuning |
| Line Up                | Lines up all robots in a specified line | Use when you need robots out of the way for testing purposes |
| One Touch Pass         | Finds the best point in front of the opponent goal to pass to and one touch it in | Don't use since it hasn't been tested / tuned |
| Our Placement          | Places the ball according to the auto-ref commands | Don't use unless trying to change ball placement code |
| Penalty                | Takes the penalty and tries to shoot | Don't use unless trying to change Our Penalty code |
| Simple Zone Midfielder | Commands 2 of the 3 offensive robots to stay at specific positions behind the ball. It just gets the out of the way so they aren't just sitting there on the field | Don't use unless there is nothing better to do |
| Tune PID               | Moves one robot back and forth to tune the motion control | Don't use unless you are trying to tune the motion control |

## Positions
These are in the tactic folder, but slightly higher level than normal tactics. They contain the AI for a single position (or group) on the field. 

Defender and Goalie are really only used on some specific restarts. Submissive Defender and Submissive Goalie are used in Defense. Defensive Forward and Simple Zone Midfielder are used in adaptive formation.

| Positions              | Description |
| ---------------------- | ----------- |
| Coach                  | The most important tactic we have. It yells strategic messages at the other robots, celebrates on goals, and follows the play from the side of the field. | 
| Defender               | Defends specific areas of the field. Only used on restarts |
| Goalie                 | Standalone goalie which does not coordinate with other robots. In most cases, it is better to add the Defense tactic, which implements the Submissive Goalie, or use the Submissive Goalie itself. |
| Submissive Defender    | Takes marking targets from the parent play. Used for more coordinated defensive plays |
| Submissive Goalie      | Takes marking targets from the parent play. Used for more coordinated defensive plays |

## Skills
There are a large number of skills, but many are not used in the higher level plays. Only Capture, Mark, Move, and Pivot/Line Kick are constantly used.

If you want to get the ball, use Capture. If you want to block an opponent robot, use Mark. To Move, use Move. To shoot, use Pivot Kick.


| Skills            | Description | Use When ... |
| ----------------- | ----------- | ------------ |
| _Kick             | Superclass to both Pivot Kick and Line Kick | Don't use as it's an abstract class |
| Aim               | Holds the ball and aims towards a specific target | Use in lower level plays to control the ball and face a specific direction |
| Angle Receive     | Receives the ball while facing a specific direction to speed up play | Use when you know what direction you want to receive the pass towards in coordinated pass. Much less consistent than the normal receive though |
| Bump              | Bumps the ball towards a certain direction | Probably best not to use as it's not very consistent |
| Capture           | Moves to intercept and collect the ball | Use whenever you want a robot to go grab the ball |
| Dribble           | Dribbles the ball in front of the robot towards a specific point | Use when you want to dribble short distances |
| Face              | Causes the robot to face a target point | Use when you know the robot should be facing a certain direction in the future. Helps improve motion control |
| Intercept         | Tries to intercept the ball at the given point as quickly as possible | Use in situations where capture would be a detriment (eg: the goalie) |
| Line Kick         | Moves into the ball and kicks | Use when you have some time to kick a stopped ball |
| Line Kick Old     | Same as line kick, but in python | Don't use |
| Line Kick Receive | Uses the same path planner as line kick to receive a pass | Don't use as it is not completely tested on real bots |
| Mark              | Places one of our robots on the given line/point/robot to block any shots/passes | Use to cover opponent robots on the defensive side of things |
| Move              | Moves a robot to a specific position | Use when you want to move robots |
| Move Direct       | Moves to a point without the path planner | Don't use unless you have a very specific reason to |
| Move Tuning       | Very simplified movement command to do PID tuning | Don't use unless you are trying to tune motion control |
| Pass Receive      | Simple receiver for a pass | Use when you just want to receive a pass normally in coordinated pass |
| Pivot Kick        | Stops the ball, circles around it, then shoots in the target direction | Use in general when you want to kick the ball during play |
| Touch Ball        | Simplified capture where the ball will roll towards the robot | Use in similar situations to one touch, where you don't want to move towards the ball while it's being passed to you |
