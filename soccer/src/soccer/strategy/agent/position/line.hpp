#pragma once

#include "position.hpp"
#include <spdlog/spdlog.h>
#include "rclcpp/rclcpp.hpp"


namespace strategy {
    class Line : public Position {

        public: 

        Line(int r_id) : Position{r_id, "line"} {}
        Line(const Position& other) : Position{other} {
            position_name_ = "Line";
        }
        ~Line() = default;
        Line(const Line&) = default;
        Line(Line&&) = default;
        Line& operator=(const Line&) = default;
        Line& operator=(Line&&) = default;


        std::string get_current_state() override {
            return "Line";
        }

        private:

        std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

        bool forward_ = true;
    };
}