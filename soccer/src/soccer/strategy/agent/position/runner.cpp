#include "runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position(r_id) {
    position_name_ = "Runner";
    current_state_ = SIDE1;
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}

Runner::State Runner::update_state() {
    State next_state = current_state_;
    // handle transitions between current state
    WorldState* world_state = this->world_state();

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
  
     if (current_state_ == SIDE1) {
      rj_geometry::Point end_position = rj_geometry::Point{3.0, 3.0};
      double distance_to_end = robot_position.dist_to(end_position);
      if (distance_to_end < 0.1) {
         next_state = SIDE2;
      }
    } else if (current_state_ == SIDE2) {
      rj_geometry::Point end_position = rj_geometry::Point{3.0, 6.0};
      double distance_to_end = robot_position.dist_to(end_position);
      if (distance_to_end < 0.1) {
         next_state = SIDE3;
      }
    } else if (current_state_ == SIDE3) {
      rj_geometry::Point end_position = rj_geometry::Point{-3.0, 6.0};
      double distance_to_end = robot_position.dist_to(end_position);
      if (distance_to_end < 0.1) {
         next_state = SIDE4;
      }
    } else if (current_state_ == SIDE4) {
      rj_geometry::Point end_position = rj_geometry::Point{-3.0, 3.0};
      double distance_to_end = robot_position.dist_to(end_position);
      if (distance_to_end < 0.1) {
         next_state = SIDE1;
      }
    }
    
    return next_state;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    
    rj_geometry::Point robot_position =
            world_state()->get_robot(true, robot_id_).pose.position();
    if (current_state_ == SIDE1) {
      auto motion_instance =
                planning::LinearMotionInstant(rj_geometry::Point{3.0, 3.0});
      intent.motion_command = planning::MotionCommand{"path_target", motion_instance, planning::FaceTarget{}, true};
      return intent;
    } else if (current_state_ == SIDE2) {
      auto motion_instance =
                planning::LinearMotionInstant(rj_geometry::Point{3.0, 6.0});
      intent.motion_command = planning::MotionCommand{"path_target", motion_instance, planning::FaceTarget{}, true};
      return intent;	
    } else if (current_state_ == SIDE3) {
      auto motion_instance =
                planning::LinearMotionInstant(rj_geometry::Point{-3.0, 6.0});
      intent.motion_command = planning::MotionCommand{"path_target", motion_instance, planning::FaceTarget{}, true};
      return intent;
    } else if (current_state_ == SIDE4) {
      auto motion_instance =
                planning::LinearMotionInstant(rj_geometry::Point{-3.0, 3.0});
      intent.motion_command = planning::MotionCommand{"path_target", motion_instance, planning::FaceTarget{}, true};
      return intent;
    }
    
    return std::nullopt;
}

}
