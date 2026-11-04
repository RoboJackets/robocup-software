#pragma once
#include <deque>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/game_state.hpp>
#include <rj_common/robot_intent.hpp>
#include <rj_common/ros_debug_drawer.hpp>
#include <rj_common/time.hpp>
#include <rj_common/world_state.hpp>
#include <rj_msgs/msg/motion_command.hpp>

#include "rj_strategy/command.hpp"
#include "rj_strategy/robot.hpp"

namespace commands {

class MoveToBallCommand : public Command {
public:
    MoveToBallCommand(std::deque<rj_msgs::msg::WorldState::SharedPtr>& world_state_cache,
                      std::shared_ptr<Robot> robot);

    void initialize() override;
    void execute() override;
    void exit(bool interrupted) override;
    bool is_done() override;

    void followBall();

private:
    std::shared_ptr<Robot> robot;

    std::deque<rj_msgs::msg::WorldState::SharedPtr>* world_state_cache;
};

}  // namespace commands