# Package Documentation

| Package Details | Entry |
| --- | --- |
| **Package name** | Cameron Lyon |
| **Assigned owner(s)** | Robojackets |
| **Reviewer(s)** | Nate, Sanat, Shourik |
| **Date started revision** | 10/16/2026 |

## 1. Package Identity and Tier

| Field | Details |
| --- | --- |
| **Plain-language purpose** | A Humble-Supported header-only helpers that turn C++ values into matching ros2 message and back, using Humbles rclpp::TypeAdapter |
| **Current responsibilities** | convert_to_ros, convert_from_ros, TypeAdapters, test_lossless_convert_ros_value, test_lossless_convert_cpp_value |
| **Out of scope** | Avoid using any third-party conversion message to transmit data. Mainly ros2 oriented conversions. |

## 2. Dependency Rules

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | rclpp, ament_cmake, ament_cmake_gtest |
| **Depended on by** | Direct package.xml dependents: common, geometry, utils, control, planning, strategy, ui |


## 3. Operational Expectations

| Field | Details |
| --- | --- |
| **Failure behavior** | Fails at compile time, no failures once compiliation works properly. |
| **Recovery behavior** | No retry, fallback, or operator action. Fix the specialization or the stack explodes. |
| **Configuration** | None |

## 4. Testing

### Test Coverage and Runtime Quality

| Field | Details |
| --- | --- |
| **Unit testing** | ros_convert_test, rj_common_convert_test, game_state_test, world_state_test, geometry_conversions_test. |
| **Coverage gaps** | None |
| **Observed outliers** | None |
