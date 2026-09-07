# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** |rj_topic_utils|
| **Assigned owner(s)** |Pooja Dayaratna|
| **Reviewer(s)** |  |
| **Date started revision** |8/31/2026|
| **Branch / tracking issue** |rj_topic_utils_cleaning|

## Purpose of This Template

Use one completed copy per package. Define what is changing, why it is changing, how the change affects the codebase, and how stability and efficiency will be verified. Complete this document with the assigned coder and reviewer before merging package changes.

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | The package defines templates for a message queue and an asynchronous message queue.|
| **Codebase role** | This is a low level utility package that is only used by two packages (rj_vision filter and rj_ui).|
| **Current responsibilities** |This package is used to create a queue of TeamColor messages in rj_vision_filter and a queue of WorldState messages and RawProtobuf messages in rj_ui. These are the only two packages that rely on it as far as I can tell (rj_control imports the async messge queue header file but never actually uses it anywhere) |
| **Out of scope** | This package shouldnt handle any logic for specific messages (meaning it should be able to work with any message type). This package should also not attempt to manage the lifecycle of any of the nodes that depend on it.  |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** |rclcpp |
| **Depended on by** | rj_vision_filter, rj_ui |

### Overlap and Coordination

| Field | Details |
| --- | --- |
| **Shared boundaries** | The standard ros2 message interfaces   |
| **Coordination needed** | rj_vision filter and rj_ui both currently depend on this package, although no one is working on them right now |

## 3. Functional Requirements

Describe what the package must do before and after the work, including behavior under failure and compatibility constraints.

| Field | Details |
| --- | --- |
| **Required behavior** | _Describe what the package must do when the work is complete._ |

### Operational Expectations

| Field | Details |
| --- | --- |
| **Failure behavior** | _Describe behavior when an upstream input, sensor, network link, radio, or other dependency fails._ |
| **Recovery behavior** | _Describe automatic recovery, retry, fallback, and operator intervention requirements._ |
| **Backward compatibility** | _Confirm topic names, message types, parameters, files, and launch behavior that must remain compatible._ |
| **Configuration** | _List required parameters, defaults, validation rules, and configuration files._ |

## 4. Stability and Efficiency

Establish measurable baselines, identify failure risks, and compare the completed package against its prior state.

### Memory and Allocation Assessment

| Field | Details |
| --- | --- |
| **Concerns** | _Identify leaks, unnecessary heap allocations, repeated allocations, large buffers, invalid indexing, uninitialized state, or repetitive work._ |

### Build-Time Comparison

| Measurement | Time Measurement | Method / Notes |
| --- | --- | --- |
| Clean selected-package build |  | `colcon build --packages-select <package>` |
| Incremental build |  | Enter the method. |
| Other |  | Enter details. |

### Test Coverage and Runtime Quality

| Field | Details |
| --- | --- |
| **Existing coverage** | _Describe current unit, integration, smoke, simulation, and regression tests._ |
| **Unit testing** | _Describe what unit tests were used on this package and how._ |
| **Coverage gaps** | _List untested algorithms, branches, failure paths, and outlier cases._ |
| **Observed outliers** | _Record flaky behavior, nondeterminism, timing spikes, or unexpected results._ |

## 5. Change Plan and Sign-Off (As per creation of this documentation)

Summarize the agreed scope, codebase impact, implementation sequence, and final decision.

| Field | Details |
| --- | --- |
| **Why is it changing?** | _State the problem, evidence, and desired outcome._ |
| **Codebase impact** | _Describe affected packages, nodes, launch files, interfaces, developers, and runtime behavior._ |
| **Success criteria** | _List objective conditions that demonstrate the work is complete and effective._ |

The final review checklist will be on ClickUp.

