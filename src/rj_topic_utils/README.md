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
| **Codebase role** | This is a utility package that is used by two packages (rj_vision filter and rj_ui).|
| **Current responsibilities** |This package is used to create a queue of TeamColor messages in rj_vision_filter and a queue of WorldState messages and RawProtobuf messages in rj_ui. These are the only two packages that rely on it as far as I can tell (rj_control imports the async messge queue header file but never actually uses it anywhere) |
| **Out of scope** | This package should be able to work with any message type.  |

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
| **Required behavior** | This package should still provide templates for message queues. but only for queueseof size >1 (queues of size 1 have been replaced with direct subscriptions so I deleted those templates since they are no longer in use) . |

### Operational Expectations

| Field | Details |
| --- | --- |
| **Failure behavior** |The package is just a set of templates for message queues so I don't think a lack of input could make it fail. 
| **Recovery behavior** |  |
| **Backward compatibility** | As of right now, the package has to be compatible with RawProtobuf msgs and the RawProtobufTopic because that is the only message type that we still use the queue for   |
| **Configuration** |

## 4. Stability and Efficiency

Establish measurable baselines, identify failure risks, and compare the completed package against its prior state.

### Memory and Allocation Assessment

| Field | Details |
| --- | --- |
| **Concerns** | I think the way that raw_vision_packet_sub uses a message queue to store raw protobuf messages is unnecessary but I was told that when we rewrite UI this will be deletedoanywas.   |

### Build-Time Comparison

| Measurement | Time Measurement | Method / Notes |
| --- | --- | --- |
| Clean selected-package build |4.76 seconds| `colcon build --packages-select <package>` |
| Incremental build |  | Enter the method. |
| Other |  | Enter details. |

### Test Coverage and Runtime Quality

| Field | Details |
| --- | --- |
| **Existing coverage** |  |
| **Unit testing** |  |
| **Coverage gaps** |  |
| **Observed outliers** |  |

## 5. Change Plan and Sign-Off (As per creation of this documentation)

Summarize the agreed scope, codebase impact, implementation sequence, and final decision.

| Field | Details |
| --- | --- |
| **Why is it changing?** | We have templates that we don't use and places where we use message queues when I don't think we need to. This clutters up the codebase, so I think changing rj_topic_utils to get rid of unnecessary templates is a good idea.   |
| **Codebase impact** | This change impacts rj_ui (since we are replacing the queue of world state msgs with a direct subscription to a world state node).  |
| **Success criteria** | Simulation behaves the same as it did before changes (including relatively the same amount of time to transition between states and similar path planning behavior.|

The final review checklist will be on ClickUp.

