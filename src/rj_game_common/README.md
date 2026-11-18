# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_game_common |
| **Assigned owner(s)** | Sanat Dhanyamraju |
| **Reviewer(s)** | Nate, Cameron |
| **Date started revision** | 11/17/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Game-level types shared across nodes: field geometry, game settings, and the current play state |
| **Current responsibilities** | `FieldDimensions`, `GameSettings`, `PlayState`, and their `rclcpp::TypeAdapter` specializations, plus `kNumShells` (number of identifiable robots per team) |
| **Out of scope** | Referee-only types (`MatchState`, `TeamInfo` live in rj_referee) and motion or world-state types (rj_motion_common) |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | rj_geometry, rj_referee_msgs, rj_vision_msgs |
| **Depended on by** | rj_benchmarking, rj_config_server, rj_control, rj_joystick, rj_motion_common, rj_planning, rj_radio, rj_referee, rj_strategy, rj_ui, rj_vision_filter, rj_vision_receiver |


## 3. Operational Expectations

| Field | Details |
| --- | --- |
| **Failure behavior** | `PlayState` logs a warning when asked to describe or transform an invalid state/restart combination |
| **Recovery behavior** | N/A |
| **Configuration** | None |

## 4. Testing

### Test Coverage and Runtime Quality

| Field | Details |
| --- | --- |
| **Unit testing** | N/A (header-only) |
| **Coverage gaps** | `PlayState` transitions and the `FieldDimensions` / `GameSettings` adapters are not directly unit tested |
| **Observed outliers** | N/A |
