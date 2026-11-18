# Package Documentation

Reusable assessment, refactor, and verification template.

| Package Details | Entry |
| --- | --- |
| **Package name** | rj_ros_common |
| **Assigned owner(s)** | Sanat Dhanyamraju |
| **Reviewer(s)** | Nate, Cameron |
| **Date started revision** | 11/17/2026 |

## 1. Package Identity and Tier

Define the package's purpose and fixed architectural classification before work starts.

| Field | Details |
| --- | --- |
| **Plain-language purpose** | Small ROS helpers shared by every node: canonical topic and service names, required-parameter declaration, and type adapters between `RJ::Time` and ROS time types |
| **Current responsibilities** | `topic_names.hpp` (topic/service names per producing node), `ros_params.hpp` (`declare_required_param`), `time_adapters.hpp` (`rclcpp::TypeAdapter` specializations for `RJ::Time` / `RJ::Seconds` to `rclcpp::Time`, `rclcpp::Duration`, and `builtin_interfaces` messages) |
| **Out of scope** | Domain types (game state belongs in rj_game_common, motion and world state in rj_motion_common) and any node logic |

## 2. Dependency Rules

Map package relationships before adding, deleting, or moving code. Dependency direction should remain downward through the tier stack.

### Dependency Summary

| Field | Details |
| --- | --- |
| **Depends on** | rj_time |
| **Depended on by** | rj_config_client, rj_config_server, rj_control, rj_joystick, rj_motion_common, rj_planning, rj_radio, rj_radio_common, rj_referee, rj_strategy, rj_ui, rj_vision_filter, rj_vision_receiver |


## 3. Operational Expectations

| Field | Details |
| --- | --- |
| **Failure behavior** | `declare_required_param` throws at node startup, naming the node and key, when a parameter is missing from the params file or has the wrong type |
| **Recovery behavior** | Add or fix the parameter in sim_params.yaml / real_params.yaml and restart the node; parameters are read once at startup |
| **Configuration** | Parameters declared through `declare_required_param` must be set in sim_params.yaml and real_params.yaml |

## 4. Testing

### Test Coverage and Runtime Quality

| Field | Details |
| --- | --- |
| **Unit testing** | N/A (header-only) |
| **Coverage gaps** | `declare_required_param` and the time adapters are not directly unit tested; the adapters are exercised through message round-trips in other packages |
| **Observed outliers** | N/A |
