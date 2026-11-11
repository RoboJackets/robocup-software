# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_planning_msgs |
| **Assigned owner(s)** | Pooja Dayaratna, Shourik Banerjee |
| **Reviewer(s)** | Sanat, Nate |
| **Date started revision** | 10/11/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Defines message, service, and action types for path planning: robot intents, motion commands, and trajectories |
| **Current responsibilities** | Responsible for planning data transferred between nodes (`RobotIntent`, `MotionCommand`, `LinearMotionInstant`, `RobotInstant`, `Trajectory`, the `PlanHypotheticalPath` service, and the `RobotMove` action) |
| **Out of scope** | This package should only define message, action, and service types for planning; low-level setpoints belong in rj_control_msgs and geometry primitives belong in rj_geometry_msgs |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | rj_geometry_msgs |
| **Depended on by** | rj_common, rj_planning, rj_strategy |


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
