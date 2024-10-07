#include "waller.hpp"

namespace strategy {

Waller::Waller(int waller_num, std::vector<u_int8_t> walling_robots) {
    defense_type_ = "Waller";
    waller_pos_ = waller_num;
    walling_robots_ = walling_robots;
}

std::optional<RobotIntent> Waller::get_task(RobotIntent intent, const WorldState* world_state,
                                            FieldDimensions field_dimensions) {
                                    
    // Creates Minimum wall radius is slightly greater than  box bounds
    // Dimension accessors should be edited when we figure out how we are doing dimensions realtime
    // from vision
    float box_w{field_dimensions.penalty_long_dist()};
    float box_h{field_dimensions.penalty_short_dist()};
    float line_w{field_dimensions.line_width()};
    double min_wall_rad{(kRobotRadius * 4.0f) + line_w +
                        hypot(static_cast<double>(box_w) / 2, static_cast<double>((box_h)))};

    SPDLOG_INFO("asdf : {}", min_wall_rad);

    auto motion_command = planning::MotionCommand{"waller"};    
    motion_command.waller_radius = min_wall_rad;
    motion_command.waller_id = waller_pos_;

    auto ball_pos = world_state->ball.position;

    uint8_t parent_id;
    if (ball_pos.x() < world_state->get_robot(true, intent.robot_id).pose.position().x()) {
        parent_id = walling_robots_[waller_pos_ - 2];
    } else {
        parent_id = walling_robots_[waller_pos_];
    }

    motion_command.waller_parent = parent_id;


    intent.motion_command = motion_command;

    return intent;
}

}  // namespace strategy
