# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_referee_msgs |
| **Assigned owner(s)** | Pooja Dayaratna, Shourik Banerjee |
| **Reviewer(s)** | Sanat, Nate |
| **Date started revision** | 10/11/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Defines message and service types for game state: referee state, team info, game settings, and the filtered world state |
| **Current responsibilities** | Responsible for game/referee data transferred between nodes (`PlayState`, `MatchState`, `TeamInfo`, `TeamColor`, `GoalieID`, `GameSettings`, `WorldState`, and the `QuickCommands` and `SetGameSettings` services) |
| **Out of scope** | This package should only define message, action, and service types for game state; per-object vision state (`RobotState`, `BallState`) belongs in rj_vision_msgs |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | rj_geometry_msgs, rj_vision_msgs |
| **Depended on by** | rj_common, rj_config_client, rj_config_server, rj_control, rj_planning, rj_radio, rj_referee, rj_strategy, rj_ui, rj_vision_filter |


## 3. Operational Expectations

| Field | Details |
| --- | --- |
| **Failure behavior** | N/A |
| **Recovery behavior** | N/A |
| **Configuration** | None |

## 4. Testing

### Test Coverage and Runtime Quality

| Field | Details |
| --- | --- |
| **Unit testing** | N/A |
| **Coverage gaps** | N/A |
| **Observed outliers** | N/A |
