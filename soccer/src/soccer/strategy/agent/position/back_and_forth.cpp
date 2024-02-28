// #include "back_and_forth.hpp"

// namespace strategy {

// BackAndForth::BackAndForth(int r_id) : Position(r_id), off_(r_id_) {
//     position_name_ = "BackAndForth";
//     current_state_ = IDLING;
//     move_on_ = false;
//     r_id_ = counter++;
//     is_running_.emplace_back(true);
// }

// std::optional<RobotIntent> BackAndForth::derived_get_task(RobotIntent intent) {
//     off_.set_globals(last_world_state_, field_dimensions_);
//     current_state_ = update_state();
//     SPDLOG_INFO(current_state_);
//     // if (r_id_ == 1) {
//     SPDLOG_INFO("Next says {}", next_);
//     // }
//     return state_to_task(intent);
// }

// BackAndForth::State BackAndForth::update_state() {
//     State next_state = current_state_;
//     WorldState* world_state = this->last_world_state_;
//     rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
//     rj_geometry::Point ball_position = world_state->ball.position;
//     double distance_to_ball = robot_position.dist_to(ball_position);

//     switch (current_state_) {
//         case IDLING: {
//             if (start() && next_ == NULLOPT) {
//                 next_state = BASIC_MOVEMENT_1;
//             }
//             if (next_ != NULLOPT && !lock_) {
//                 next_state = next_;
//                 lock_ = true;
//             }
//             break;
//         }

//         case BASIC_MOVEMENT_1: {
//             if (proceed()) {
//                 move_on_ = false;
//                 next_state = BASIC_MOVEMENT_2;
//             }
//             break;
//         }

//         case BASIC_MOVEMENT_2: {
//             if (proceed()) {
//                 move_on_ = false;
//                 next_state = BASIC_MOVEMENT_3;
//                 next_ = BASIC_MOVEMENT_3;
//             }
//             break;
//         }

//         case BASIC_MOVEMENT_3: {
//             if (proceed()) {
//                 move_on_ = false;
//                 next_state = BASIC_MOVEMENT_2;
//                 next_ = BASIC_MOVEMENT_2;
//             }
//             break;
//         }

//         default:
//             next_state = IDLING;
//     }

//     return next_state;
// }

// std::optional<RobotIntent> BackAndForth::state_to_task(RobotIntent intent) {
//     WorldState* world_state = this->last_world_state_;
//     rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
//     switch (current_state_) {
//         case IDLING: {
//             break;
//         }

//         case BASIC_MOVEMENT_1: {
//             rj_geometry::Point target_pos = rj_geometry::Point(-.24 + .6 * robot_id_, 1.75);
//             planning::LinearMotionInstant target{target_pos};
//             auto motion_cmd = planning::MotionCommand{"path_target", target};
//             intent.motion_command = motion_cmd;
//             intent.is_active = true;
//             break;
//         }

//         case BASIC_MOVEMENT_2: {
//             rj_geometry::Point target_pos = rj_geometry::Point(-.24 + .6 * robot_id_, 7.0);
//             planning::LinearMotionInstant target{target_pos};
//             planning::PathTargetFaceOption face_option{
//                 planning::FacePoint{rj_geometry::Point{0, 4.5}}};
//             auto motion_cmd = planning::MotionCommand{"path_target", target, face_option, true};
//             intent.motion_command = motion_cmd;
//             intent.is_active = true;
//             break;
//         }

//         case BASIC_MOVEMENT_3: {
//             rj_geometry::Point target_pos = rj_geometry::Point(-.24 + .6 * robot_id_, 2);
//             planning::LinearMotionInstant target{target_pos};
//             planning::PathTargetFaceOption face_option{
//                 planning::FacePoint{rj_geometry::Point{-3, 4.5}}};
//             auto motion_cmd = planning::MotionCommand{"path_target", target, face_option, true};
//             intent.motion_command = motion_cmd;
//             intent.is_active = true;
//             break;
//         }
//     }

//     return intent;
// }

// bool BackAndForth::proceed() {
//     bool done = check_is_done();
//     if (done || move_on_) {
//         is_running_.at(r_id_) = false;
//         move_on_ = true;
//     } else {
//         is_running_.at(r_id_) = true;
//     }
//     return (std::find(is_running_.begin(), is_running_.end(), true) == is_running_.end());
// }

// bool BackAndForth::start() {
//     is_running_.at(r_id_) = false;
//     return (std::find(is_running_.begin(), is_running_.end(), true) == is_running_.end());
// }

// bool BackAndForth::derived_check() { return off_.derived_check_is_done(); }

// }  // namespace strategy