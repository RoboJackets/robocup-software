#pragma once 
#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rj_geometry/point.hpp>


#include <rj_geometry/point.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {

    class Runner : public Position {
        public:
            ~Runner() override = default;
            Runner(int r_id);
            Runner(const Position& other);

        private:
            enum State {
                CENTER,
                DRIVING_TO_TOP_LEFT,
                DRIVING_TO_TOP_RIGHT,
                DRIVING_TO_BOTTOM_RIGHT,
                DRIVING_TO_BOTTOM_LEFT,
            };

            State next_state();
            State current_state_ = State::CENTER;
            string state_to_name(State state){
                switch(state){
                    case CENTER:
                        return "Center";
                    case DRIVING_TO_TOP_LEFT:
                        return "Driving to top left";
                    case DRIVING_TO_TOP_RIGHT:
                        return "Driving to top right";
                    case DRIVING_TO_BOTTOM_RIGHT:
                        return "Driving to bottom right";
                    case DRIVING_TO_BOTTOM_LEFT:
                        return "Driving to bottom left";
                }
            }
            std::optional<RobotIntent> state_to_task(RobotIntent intent);
            std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

            std::string get_current_state();


    };
}