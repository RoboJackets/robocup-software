# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_geometry_msgs |
| **Assigned owner(s)** | Shourik Banerjee |
| **Reviewer(s)** | Nathanial Wert, Sanat Dhanyamraju |
| **Date started revision** | 9/1/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Defines the ROS2 message types for basic geometry primitives. The messages are used to send geometric data between different nodes. |
| **Current responsibilities** | Define .msg files for basic geometry primitives along with .msg for ShapeSet which is an aggregate of a collection of basic shapes. |
| **Out of scope** | No C++ classes or actual geometry logic |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | None |
| **Depended on by** | rj_geometry , rj_msgs , rj_utils , rj_drawing_msgs |


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
| **Unit testing** | None and none applicable |
| **Coverage gaps** | None |
| **Observed outliers** | None |
