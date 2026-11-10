# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_drawing_msgs |
| **Assigned owner(s)** | Sanat Dhanyamraju |
| **Reviewer(s)** | Nate Wert, Pooja Dayaratna |
| **Date started revision** | 10/11/26 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Defines the ROS2 message types for other packages to send debug drawings to the UI. |
| **Current responsibilities** | Responsible for the .msg files for all drawable components. |
| **Out of scope** | Does not contain any C++ classes or drawing logic |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | rj_geometry_msgs |
| **Depended on by** | rj_common |


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
| **Coverage gaps** | None |
| **Observed outliers** | None |
