# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_geometry_msgs |
| **Assigned owner(s)** | Shourik Banerjee |
| **Reviewer(s)** | Nathanial Wert, Sanat Dhanyamraju |
| **Date started revision** | 9/1/2026 |
| **Branch / tracking issue** | geometry_msgs_doc_update |

## Purpose of This Template

Use one completed copy per package. Define what is changing, why it is changing, how the change affects the codebase, and how stability and efficiency will be verified. Complete this document with the assigned coder and reviewer before merging package changes.

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Defines the ROS2 message types for basic geometry primitives. The messages are used to send geometric data between different nodes. |
| **Codebase role** | It is defined at the base of the geometry stack to let geometry data be transmitted across nodes. Consumed almost exclusively by rj_geometry. |
| **Current responsibilities** | Define .msg files for basic geometry primitives along with .msg for ShapeSet which is an aggregate of a collection of basic shapes. |
| **Out of scope** | No C++ classes or actual geometry logic |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | _Leaf node_ |
| **Depended on by** | rj_geometry , rj_msgs , rj_utils , rj_drawing_msgs |

### Overlap and Coordination

| Field | Details |
| --- | --- |
| **Shared boundaries** | Each message type that exists in this package has a corresponding class that exists in rj_geometry |
| **Coordination needed** | _N/A_ |

## 3. Functional Requirements

Describe what the package must do before and after the work, including behavior under failure and compatibility constraints.

| Field | Details |
| --- | --- |
| **Required behavior** | Must still correctly define ROS2 serializable messages for geometry shapes preserving layouts so that existing RosConverters in rj_geometry still continue to work. |

### Operational Expectations

| Field | Details |
| --- | --- |
| **Failure behavior** | N/A |
| **Recovery behavior** | N/A |
| **Backward compatibility** | All existing message types and their field names/types must remain unchanged. Each has a corresponding RosConverter specialization in rj_geometry that relies on these exact fields. If a message rename must occur, it must also be changed in RosConverter. |
| **Configuration** | None |

## 4. Stability and Efficiency

Establish measurable baselines, identify failure risks, and compare the completed package against its prior state.

### Memory and Allocation Assessment

| Field | Details |
| --- | --- |
| **Concerns** | None |

### Build-Time Comparison

| Measurement | Time Measurement | Method / Notes |
| --- | --- | --- |
| Clean selected-package build | 5 seconds | `colcon build --packages-select rj_geometry_msgs` |
| Incremental build | 1 second | `colcon build --packages-select rj_geometry_msgs` |
| Other |  | Enter details. |

### Test Coverage and Runtime Quality

| Field | Details |
| --- | --- |
| **Existing coverage** | None |
| **Unit testing** | None and none applicable |
| **Coverage gaps** | None |
| **Observed outliers** | None |

## 5. Change Plan and Sign-Off (As per creation of this documentation)

Summarize the agreed scope, codebase impact, implementation sequence, and final decision.

| Field | Details |
| --- | --- |
| **Why is it changing?** | No changes, just documentation. |
| **Codebase impact** | No changes made now but any additional changes would affect rj_geometry, or if additional shapes are added to rj_geometry than they can need an associated message here. |
| **Success criteria** | All nine geometry messages are confirmed live via production call sites, no schema changes are made, the package builds cleanly, and documentation accurately reflects these findings. |

The final review checklist will be on ClickUp.
