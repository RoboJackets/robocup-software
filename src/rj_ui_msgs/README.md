# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_ui_msgs |
| **Assigned owner(s)** | Pooja Dayaratna, Shourik Banerjee |
| **Reviewer(s)** | Sanat, Nate |
| **Date started revision** | 10/11/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Defines message and service types sent from the UI, such as simulator placement and position overrides |
| **Current responsibilities** | Responsible for UI-originated data transferred between nodes (`BallPlacement`, `RobotPlacement`, `OverridePosition`, the `SimPlacement` service) |
| **Out of scope** | This package should only define message, action, and service types for the UI; debug drawing types belong in rj_drawing_msgs |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | rj_geometry_msgs |
| **Depended on by** | rj_radio, rj_strategy, rj_ui |


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
