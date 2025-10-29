#include "rj_strategy/coordinator/marking.hpp"

namespace strategy {

Marking::Marking()
    : Coordinator("marking_srv", "marking_data", "marking_node") {
    // Subscribe to world state
    marking_list_.fill(kInvalidRobotId); // initializes to no valid markers
    enemey_to_friends_.fill(kInvalidRobotId);
    danger_score_.fill(std::numeric_limits<double>::infinity()); // everyone starts with an infinite danger score
    num_markers_ = 0;
    world_state_sub_ = this->create_subscription<rj_msgs::msg::WorldState>(
        vision_filter::topics::kWorldStateTopic, rclcpp::QoS(1),
        [this](rj_msgs::msg::WorldState::SharedPtr world_state) {  // NOLINT
            last_world_state_ = rj_convert::convert_from_ros(*world_state);
            publish_marking_list();
        });
}

void Marking::service_callback(RequestPtr request, ResponsePtr response) {
    if (request->join == false) {
        bool marking = (marking_list_[request->robot_id] != kInvalidRobotId);
        if (marking) {
            uint8_t enemey_id = marking_list_[request->robot_id];
            marking_list_[request->robot_id] = kInvalidRobotId;
            enemey_to_friends_[enemey_id] = kInvalidRobotId;
            num_markers_--;
            // replace if possible
            const auto& enemey_robot = last_world_state_.get_robot(false, enemey_id);
            double min = std::numeric_limits<double>::infinity();
            uint8_t waiting_robot_id = kInvalidRobotId;
            for (size_t i = 0; i < queue_.size(); ++i) {
                const auto& i_robot = last_world_state_.get_robot(true, queue_[i]);
                double distance = i_robot.pose.position().dist_to(enemey_robot.pose.position());
                if (distance < min) {
                    min = distance;
                    waiting_robot_id = queue_[i];
                }
            }
            if (kInvalidRobotId != waiting_robot_id) {
                queue_.erase(std::remove(queue_.begin(), queue_.end(), waiting_robot_id), queue_.end());
                marking_list_[waiting_robot_id] = enemey_id;
                enemey_to_friends_[enemey_id] = waiting_robot_id;
                num_markers_++;
            }
        } else {
            queue_.erase(std::remove(queue_.begin(), queue_.end(), request->robot_id), queue_.end());
        }
        response->success = true;
        return;
    }
    if (num_markers_ < kMaxMarkers) {
        uint8_t robotInPossession = kInvalidRobotId;
        double min_dist_to_ball = std::numeric_limits<double>::infinity();
        const auto& ball_pos = last_world_state_.ball.position;
        for (uint8_t i = 0; i < kNumShells; i++) {
            const auto& robot = last_world_state_.get_robot(false, i);
            if (!robot.visible) {
                continue;
            }
            double dist_to_ball = ball_pos.dist_to(robot.pose.position());
            if (dist_to_ball < min_dist_to_ball && dist_to_ball < 0.3) {
                robotInPossession = i;
                min_dist_to_ball = dist_to_ball;
            }
        }


        uint8_t most_dangerous = kInvalidRobotId;
        double min = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < danger_score_.size(); ++i) {
            if (enemey_to_friends_[i] != kInvalidRobotId || i == robotInPossession) {
                continue;
            }
            if (danger_score_[i] < min) {
                most_dangerous = i;
                min = danger_score_[i];
            }
        }
        if (most_dangerous != kInvalidRobotId) {
            enemey_to_friends_[most_dangerous] = request->robot_id;
            marking_list_[request->robot_id] = most_dangerous;
            num_markers_++;
        } else {
            queue_.push_back(request->robot_id);
        }
    } else {
        // should we kick someone out
        double better_distance = 0;
        uint8_t kick_out_this_robot_id = kInvalidRobotId;
        const auto& robot_requesting = last_world_state_.get_robot(true, request->robot_id);
        for (size_t i = 0; i < marking_list_.size(); ++i) {
            if (marking_list_[i] != kInvalidRobotId) {
                uint8_t enemey_id = marking_list_[i];
                const auto& i_robot = last_world_state_.get_robot(true, i);
                const auto& enemey_robot = last_world_state_.get_robot(false, enemey_id);
                double dist = (i_robot.pose.position().dist_to(enemey_robot.pose.position())) - (robot_requesting.pose.position().dist_to(enemey_robot.pose.position()));
                if (dist > better_distance) {
                    better_distance = dist;
                    kick_out_this_robot_id = i;
                }
            }
        }
        if (kick_out_this_robot_id != kInvalidRobotId) {
            uint8_t enemey_id = marking_list_[kick_out_this_robot_id];
            marking_list_[kick_out_this_robot_id] = kInvalidRobotId;
            enemey_to_friends_[enemey_id] = request->robot_id;
            marking_list_[request->robot_id] = enemey_id;
        } else {
            queue_.push_back(request->robot_id);
        }
    }

    response->success = true;
}

void Marking::publish_marking_list() {
    // make this run on a timer
    update_danger_scores();

    // find the guy with possession of the ball if exists so we don't mark them
    uint8_t robotInPossession = kInvalidRobotId;
    double min_dist_to_ball = std::numeric_limits<double>::infinity();
    const auto& ball_pos = last_world_state_.ball.position;
    for (uint8_t i = 0; i < kNumShells; i++) {
        const auto& robot = last_world_state_.get_robot(false, i);
        if (!robot.visible) {
            continue;
        }
        double dist_to_ball = ball_pos.dist_to(robot.pose.position());
        if (dist_to_ball < min_dist_to_ball && dist_to_ball < 0.3) {
            robotInPossession = i;
            min_dist_to_ball = dist_to_ball;
        }
    }

    if (robotInPossession != kInvalidRobotId) {
        SPDLOG_INFO("Robot in possession is {}", robotInPossession);
    }


    // finding most dangerous of non-marked robots
    uint8_t most_dangerous = kInvalidRobotId;
    double min = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < danger_score_.size(); ++i) {
        if (enemey_to_friends_[i] != kInvalidRobotId || i == robotInPossession) {
            continue;
        }
        if (danger_score_[i] < min) {
            most_dangerous = i;
            min = danger_score_[i];
        }
    }

    for (size_t i = 0; i < marking_list_.size(); ++i) {
        if (marking_list_[i] == robotInPossession && robotInPossession != kInvalidRobotId) {
            enemey_to_friends_[robotInPossession] = kInvalidRobotId;
            marking_list_[i] = most_dangerous;
            enemey_to_friends_[most_dangerous] = i;
        }
    }

    if (most_dangerous != kInvalidRobotId) {
        uint8_t not_dangerous_robot_id = kInvalidRobotId;
        double max_danger_sub = 0.0;
        for (size_t i = 0; i < marking_list_.size(); ++i) {
            if (marking_list_[i] != kInvalidRobotId) {
                uint8_t enemey_id = marking_list_[i];
                double danger_sub = danger_score_[enemey_id] - danger_score_[most_dangerous];
                if (danger_sub > max_danger_sub) {
                    max_danger_sub = danger_sub;
                    not_dangerous_robot_id = enemey_id;
                }
            }
        }
        // seeing if most dangerous of non-marked robots is significantly more dangerous than any marked robot
        if (max_danger_sub > kSuperDangerSub && not_dangerous_robot_id != kInvalidRobotId) {
            uint8_t friend_id = enemey_to_friends_[not_dangerous_robot_id];
            enemey_to_friends_[not_dangerous_robot_id] = kInvalidRobotId;
            marking_list_[friend_id] = most_dangerous;
            enemey_to_friends_[most_dangerous] = friend_id;
        }
    }

    publisher_->publish(rj_msgs::msg::Marking().set__mark_robot_ids(marking_list_));
}

void Marking::update_danger_scores() {

    // danger score calculation is distance_to_ball * constant + distance_to_goal * constant - distance_from_our_closest_robot * constant - danger_angle * constant
    // lower danger score is more dangerous

    const auto& ball_pos = last_world_state_.ball.position;
    const auto& goal_loc = field_dimensions_.our_goal_loc();
    const auto& field_center = field_dimensions_.center_field_loc();

    for (uint8_t i = 0; i < kNumShells; i++) {
        const auto& robot = last_world_state_.get_robot(false, i);
        if (!robot.visible) {
            continue;
        }
        double dist_to_ball = ball_pos.dist_to(robot.pose.position());
        double dist_to_goal = robot.pose.position().dist_to(goal_loc);

        double min = std::numeric_limits<double>::infinity();
        for (uint8_t j = 0; j < kNumShells; j++) {
            const auto& i_friend = last_world_state_.get_robot(true, j);
            if (!robot.visible) {
                continue;
            }
            double dist = robot.pose.position().dist_to(i_friend.pose.position());
            if (dist < min) {
                min = dist;
            }
        }

        double angle_between = 0.0; // Default to 0 (not dangerous)
        // Check if beyond midfield

        bool onOurSide = false;
        if (goal_loc.y() < field_center.y()) {
            onOurSide = robot.pose.position().y() < field_center.y();
        } else {
            onOurSide = robot.pose.position().y() > field_center.y();
        }

        if (onOurSide) {
            const auto& vec_goal_to_center = field_center - goal_loc;
            const auto& vec_goal_to_robot = robot.pose.position() - goal_loc;

            double cosTheta = vec_goal_to_center.dot(vec_goal_to_robot) /
                                (vec_goal_to_center.mag() * vec_goal_to_robot.mag());

            if (cosTheta > 1.0) cosTheta = 1.0;
            if (cosTheta < -1.0) cosTheta = -1.0;
            double central_angle = std::abs(std::acos(cosTheta)); // [0, PI/2]
            // Normalize
            double normalized_danger = (M_PI_2 - central_angle) / M_PI_2;
            if (normalized_danger < 0.0) normalized_danger = 0.0; // Clamp

            // Scales angles so that more central angles close together and more sideline are futher apart
            const double kDangerAngleExponent = 0.25;
            angle_between = std::pow(normalized_danger, kDangerAngleExponent);
        }

        double danger_score = dist_to_ball * kDangerDistToBall + dist_to_goal * kDangerDistToGoal - min * kDangerDistToOurRobots - angle_between * kDangerAngle;

        danger_score_[i] = danger_score;
    }

    for (size_t i = 0; i < 6; ++i) {
        SPDLOG_INFO("Robot {} has danger score {}", i, danger_score_[i]);
    }
}


}  // namespace strategy

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<strategy::Marking>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
