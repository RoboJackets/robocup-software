#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/planning/instant.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/robot_move.hpp>

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position/seeker.hpp"
#include "rj_strategy/agent/position_utils.hpp"

namespace strategy {
	class Runner : public Position {
	public:
		Runner(int r_id);
		~Runner() override = default;
		Runner(Position&& other);
		// communication::PosAgentResponseWrapper receive_communication_request(
       	// 	communication::AgentPosRequestWrapper request) override;
		
		// void receive_communication_response(communication::AgentPosResponseWrapper response) override;

		// void derived_acknowledge_pass() override;
		// void derived_pass_ball() override;
		// void derived_acknowledge_ball_in_transit() override;

		std::string get_current_state() override;

		std::string get_state_name() const override {
			return std::string(state_to_name(current_state_));
		}

	private:
		std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
		
		enum State {
			DEFAULT,
			SIDE_ONE,
			SIDE_TWO,
			SIDE_THREE,
			SIDE_FOUR
		};

		State next_state();
		State current_state_ = State::DEFAULT;
		RJ::Time last_time_;

		std::optional<RobotIntent> state_to_task(RobotIntent intent);

		static constexpr std::string_view state_to_name(State s) {
			switch (s) {
				case DEFAULT:
					return "DEFAULT";
				case SIDE_ONE:
					return "SIDE_ONE";
				case SIDE_TWO:
					return "SIDE_TWO";
				case SIDE_THREE:
					return "SIDE_THREE";
				case SIDE_FOUR:
					return "SIDE_FOUR";

			}
		}

		void reset_timeout() {
			last_time_ = RJ::now();
		}
	};
}