# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_control_msgs |
| **Assigned owner(s)** | Pooja Dayaratna, Shourik Banerjee |
| **Reviewer(s)** | Sanat, Nate |
| **Date started revision** | 10/11/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Defines message types for low-level robot control setpoints sent to the robots |
| **Current responsibilities** | Responsible for motion and manipulator setpoints transferred between nodes (`MotionSetpoint`, `ManipulatorSetpoint`) |
| **Out of scope** | This package should only define message, action, and service types for control setpoints; planning-level commands and trajectories belong in rj_planning_msgs |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | None |
| **Depended on by** | rj_common, rj_joystick, rj_planning, rj_radio |


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
