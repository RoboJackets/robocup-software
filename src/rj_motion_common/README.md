# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_motion_common |
| **Assigned owner(s)** | Sanat Dhanyamraju |
| **Reviewer(s)** | Nate, Cameron |
| **Date started revision** | 11/17/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Motion and world-state types shared by strategy, planning, control, and radio |
| **Current responsibilities** | `WorldState` / `RobotState` / `BallState` (including ball motion prediction), `RobotInstant` / `LinearMotionInstant`, `Trajectory`, `MotionCommand`, `RobotIntent`, and `RosDebugDrawer`, with their `rclcpp::TypeAdapter` specializations |
| **Out of scope** | Planning algorithms (rj_planning), control loops (rj_control), and game or referee state (rj_game_common, rj_referee) |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | rj_drawing_msgs, rj_game_common, rj_geometry, rj_planning_msgs, rj_ros_common, rj_time, rj_vision_msgs |
| **Depended on by** | rj_control, rj_planning, rj_radio, rj_referee, rj_strategy, rj_ui, rj_vision_filter |


## 3. Operational Expectations

| Field | Details |
| --- | --- |
| **Failure behavior** | `Trajectory` throws when read while empty, when an instant is appended at or before its last instant, when splicing trajectories that do not meet, or when serializing invalid angles; other types are plain data |
| **Recovery behavior** | N/A |
| **Configuration** | Ball prediction takes the ball decay constant from the caller (`ball_decay_constant` in sim_params.yaml / real_params.yaml) |

## 4. Testing

### Test Coverage and Runtime Quality

| Field | Details |
| --- | --- |
| **Unit testing** | `ball_test` (gtest) covers `BallState` prediction; `Trajectory` is covered by `trajectory_test` in rj_planning |
| **Coverage gaps** | `MotionCommand`, `RobotIntent`, and `WorldState` conversions to and from messages are not unit tested |
| **Observed outliers** | N/A |
