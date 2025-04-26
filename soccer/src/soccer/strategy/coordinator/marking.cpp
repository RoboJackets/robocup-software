#include "marking.hpp"

#include <algorithm>  // for std::any_of
#include <limits>

#include <rj_constants/topic_names.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/marking.hpp>

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
            int enemey_id = marking_list_[request->robot_id];
            marking_list_[request->robot_id] = kInvalidRobotId;
            enemey_to_friends_[enemey_id] = kInvalidRobotId;
            num_markers--;
            // replace if possible
            const auto& enemey_robot = last_world_state_.get_robot(false, enemey_id);
            double min = std::numeric_limits<double>::infinity();
            int waiting_robot_id = kInvalidRobotId;
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
                num_markers++;
            }
        } else {
            queue_.erase(std::remove(queue_.begin(), queue_.end(), request->robot_id), queue_.end());
        }
        response->success = true;
        return;
    }
    if (num_markers_ < kMaxMarkers) {
        int most_dangerous = kInvalidRobotId;
        double min = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < danger_score_.size(); ++i) {
            if (enemey_to_friends_[i] != kInvalidRobotId) {
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
            num_markers++;
        } else {
            queue_.push_back(request->robot_id);
        }
    } else {
        // should we kick someone out
        double better_distance = 0;
        int kick_out_this_robot_id = kInvalidRobotId;
        const auto& robot_requesting = last_world_state_.get_robot(true, request->robot_id);
        for (size_t i = 0; i < marking_list_.size(); ++i) {
            if (marking_list_[i] != kInvalidRobotId) {
                int enemey_id = marking_list_[i];
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
            int enemey_id = marking_list_[kick_out_this_robot_id];
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

    // reshuffle, only change one because on timer so will get others later
    int most_dangerous = kInvalidRobotId;
    double min = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < danger_score_.size(); ++i) {
        if (enemey_to_friends_[i] != kInvalidRobotId) {
            continue;
        }
        if (danger_score_[i] < min) {
            most_dangerous = i;
            min = danger_score_[i];
        }
    }
    if (most_dangerous != kInvalidRobotId) {
        int not_dangerous_robot_id = kInvalidRobotId;
        double max_danger_sub = 0.0;
        for (size_t i = 0; i < marking_list_.size(); ++i) {
            if (marking_list_[i] != kInvalidRobotId) {
                int enemey_id = marking_list_[i];
                double danger_sub = danger_score_[enemey_id] - danger_score_[most_dangerous];
                if (danger_sub > max_danger_sub) {
                    max_danger_sub = danger_sub;
                    not_dangerous_robot_id = enemey_id;
                }
            }
        }
        if (not_dangerous_robot_id != kInvalidRobotId) {
            int friend_id = enemey_to_friends_[not_dangerous_robot_id];
            enemey_to_friends_[not_dangerous_robot_id] = kInvalidRobotId;
            marking_list_[friend_id] = most_dangerous;
            enemey_to_friends_[most_dangerous] = friend_id;
        }
    }

    publisher_->publish(rj_msgs::msg::Marking().set__mark_robot_ids(marking_list_));
}

void Marking::update_danger_scores() {
    // const auto& ball_pos = last_world_state_.ball.position;

    // for (uint8_t i = 0; i < kNumShells; i++) {
    //     const auto& robot = last_world_state_.get_robot(false, i);
    //     double dist_to_ball_ = ball_pos.dist_to(robot.pose.position());
    //     //double dist_to_goal_ = robot.pose.position().dist_to(field_dimensions_.our_goal_loc());

    // }
}


}  // namespace strategy
