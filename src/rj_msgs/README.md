# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** |rj_msgs  |
| **Assigned owner(s)** | Pooja Dayaratna, Shourik Banerjee |
| **Reviewer(s)** | Sanat, Nate |
| **Date started revision** | 10/11/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Defines message, action, and service types for the entire codebase (with the exception of messages that are specific to geometry and drawing) |
| **Current responsibilities** | Responsible for all data transferred between nodes that is NOT specific to geometry or drawing |
| **Out of scope** | This package should only define message, action, and service types; additionally, this package must not attempt to define message types for messages that are related to goemetry and drawing, since we have the rj_geometry_msgs and rj_drawing_msgs packages for that |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | rj_geometry_msgs, std_msgs |
| **Depended on by** |rj_benchmarking, rj_common, rj_config_client, rj_config_server, rj_control, rj_joystick, rj_param_utils, rj_radio, rj_referee, rj_strategy, rj_ui, rj_utils, rj_vision_filter, rj_vision_reciever|


## 3. Operational Expectations

| Field | Details |
| --- | --- |
| **Failure behavior** | N/A |
| **Recovery behavior** | N/A |
| **Configuration** | ManipulatorSetpoint.msg and RobotIntent.msg contain some default values; no other messages, actions, or services do |

## 4. Testing

### Test Coverage and Runtime Quality

| Field | Details |
| --- | --- |
| **Unit testing** | for each message type, I searched the codebase for references to that message and deleted it if there were none. Then I ran colcon build and ensured that the stack built without error. Then I launched the simulator and ensured that deleting these messages didn't cause any issues. |
| **Coverage gaps** | N/A |
| **Observed outliers** | N/A |
