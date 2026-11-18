# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_time |
| **Assigned owner(s)** | Sanat Dhanyamraju |
| **Reviewer(s)** | Nate, Cameron |
| **Date started revision** | 11/17/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Defines the clock and duration types used across the stack (`RJ::Time`, `RJ::Seconds`) using only `std::chrono`, so any package can get the current time without depending on ROS |
| **Current responsibilities** | `RJ::Time` / `RJ::Seconds` types, `RJ::now()`, duration helpers (`num_seconds`, `num_microseconds`), `RJ::Time` +/- `RJ::Seconds` operators, and printing of `RJ::Seconds` |
| **Out of scope** | Must not depend on ROS; conversions between `RJ::Time` and ROS time/duration types belong in rj_ros_common |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | None |
| **Depended on by** | rj_control, rj_motion_common, rj_planning, rj_radio, rj_ros_common, rj_strategy, rj_ui, rj_vision_filter |


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
| **Unit testing** | N/A (header-only type aliases and helpers) |
| **Coverage gaps** | Time helpers are not directly unit tested; they are exercised through rj_motion_common, rj_planning, and rj_vision_filter tests |
| **Observed outliers** | N/A |
