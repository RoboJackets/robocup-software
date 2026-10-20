#pragma once

#include <string> 

#include <rclcpp/rclcpp.hpp>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/motion_command.hpp>
#include <rj_geometry/point.hpp>

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position/role_interface.hpp"

namespace strategy {
    class CornerKicker : public Position {
        public:
            CornerKicker(int r_id);
            ~CornerKicker() = default;

            // essential move constructor from Position
            CornerKicker(Position&& other); 

            /**
            * @brief Does nothing; this position is a special case
            */
            void derived_acknowledge_pass() override;
            /**
            * @brief Does nothing; this position is a special case
            */
            void derived_pass_ball() override;
            /**
            * @brief Does nothing; this position is a special case
            */
            void derived_acknowledge_ball_in_transit() override;

            // debug/logging tool
            std::string get_current_state() override;

        private:
            std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

            // 1. states
            enum State {
                APPROACHING, // go behind ball
                LINE_UP,     // align
                KICKING      // kick
            };

            // 2. constants
            static constexpr double kOwnBallRadius{kRobotRadius + 0.1};
            // Settle distance behind the ball while lining up
            static constexpr double kApproachDist{0.20}; 
            // Tolerance to consider the robot lined up with target heading
            static constexpr double kAngleThreshold{0.05}; // radians (~3 deg)

            State update_state();


            //============================== 
            // 
            double distance_to_ball() const {
                return last_world_state_->ball.position.dist_to(
                    last_world_state_->get_robot(true, robot_id_).pose.position());
                    
                }
                
                /**
                * @brief Calculates the distance of vector from other team's closest robot
                */
                rj_geometry::Point calculate_best_pass_target() const;
                
                rj_geometry::Point target_;
                
                std::optional<RobotIntent> state_to_task(RobotIntent intent);
                
                State latest_state_ = APPROACHING;
                //============================== 

    };
} // name space strategy