#pragma once
 #include <rclcpp/rclcpp.hpp>
 #include <rj_msgs/msg/world_state.hpp>
 #include "position.hpp"
 #include "rj_geometry/point.hpp"

 namespace strategy {
    class Blocker {
        public:
            static std::optional<RobotIntent> get_task(RobotIntent intent, const WorldState* const world_state, const double factor);
    };
 }