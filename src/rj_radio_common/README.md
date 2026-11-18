# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_radio_common |
| **Assigned owner(s)** | Sanat Dhanyamraju |
| **Reviewer(s)** | Nate, Cameron |
| **Date started revision** | 11/17/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Radio types shared between rj_radio and rj_ui |
| **Current responsibilities** | `RobotStatus` (status reported back from a robot) and its `rclcpp::TypeAdapter` to `rj_radio_msgs/RobotStatus` |
| **Out of scope** | Radio transport and packet conversion (rj_radio) and status display (rj_ui) |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | rj_geometry, rj_radio_msgs, rj_ros_common |
| **Depended on by** | rj_radio, rj_ui |


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
| **Unit testing** | N/A (header-only) |
| **Coverage gaps** | The `RobotStatus` adapter is not unit tested |
| **Observed outliers** | N/A |
