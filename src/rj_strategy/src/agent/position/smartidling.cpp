#include "rj_strategy/agent/position/smartidling.hpp"

namespace strategy {

SmartIdle::SmartIdle(int r_id) : Position{r_id, "SmartIdle"} {}

SmartIdle::SmartIdle(const Position& other) : Position{other} {}

SmartIdle::SmartIdle(int r_id, std::shared_ptr<ClientHandles> clientHandles)
    : Position(r_id, "SmartIdle"), clientHandles_{clientHandles} {}

SmartIdle::SmartIdle(const Position& other, std::shared_ptr<ClientHandles> clientHandles)
    : Position{other}, clientHandles_{clientHandles} {}

std::string SmartIdle::get_current_state() { return "SmartIdle"; }

/**
 * @brief Does nothing; this position is a special case
 */
void SmartIdle::derived_acknowledge_pass() {}
/**
 * @brief Does nothing; this position is a special case
 */
void SmartIdle::derived_pass_ball(){

};
/**
 * @brief Does nothing; this position is a special case
 */
void SmartIdle::derived_acknowledge_ball_in_transit() {}

std::optional<RobotIntent> SmartIdle::derived_get_task(RobotIntent intent) {
    intent.motion_command = planning::MotionCommand{"halt"};
    return intent;
};

}  // namespace strategy
