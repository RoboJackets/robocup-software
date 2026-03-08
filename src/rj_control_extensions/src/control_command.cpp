#include "rj_control_extensions/control_command.hpp"

namespace control {

void ControlCommand::as_sim_command(int robot_id, RobotCommand* command) const {
    command->set_id(robot_id);

    if (trigger_mode_ == TriggerMode::STAND_DOWN) {
        command->set_kick_speed(0);
        command->set_kick_angle(0);   
    } else {
        if (shoot_mode_ == ShootMode::KICK) {
            command->set_kick_speed(kick_strength_);
            command->set_kick_angle(0);
        } else {
            command->set_kick_speed(kick_strength_);
            command->set_kick_angle(40);
        }
    }

    auto* move_command = command->mutable_move_command()->mutable_local_velocity();
    move_command->set_forward(static_cast<float>(velocity_.linear().y()));
    move_command->set_left(-static_cast<float>(velocity_.linear().x()));
    move_command->set_angular(static_cast<float>(velocity_.angular()));

    command->set_dribbler_speed(static_cast<float>(dribble_speed_ * 255.0));
}

} // namespace control