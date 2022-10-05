#include "coach_node.hpp"

using std::placeholders::_1;

CoachNode::CoachNode(const rclcpp::NodeOptions& options) : Node("coach_node", options) {
    coach_pub_ = this->create_publisher<rj_msgs::msg::CoachStateInterpretation>("/strategy/coach", 10);
    play_state_change_timer_ = this->create_wall_timer(
        100ms, [this]() { check_for_play_state_change();   });

    play_state_sub_ = this->create_subscription<rj_msgs::msg::PlayState>(
        "/referee/play_state", 10,
        [this](rj_msgs::msg::PlayState::SharedPtr msg) { play_state_callback(msg); });

    world_state_sub_ = this->create_subscription<rj_msgs::msg::WorldState>(
        "/vision_filter/world_state", 10,
        [this](rj_msgs::msg::WorldState::SharedPtr msg) { world_state_callback(msg); });

    current_play_state_.state = PlayState::State::Halt;
    current_play_state_.restart = PlayState::Restart::Kickoff;
    current_play_state_.our_restart = true;

    rj_geometry_msgs::msg::Point temp_point;
    temp_point.x = -1;
    temp_point.y = -1;
    current_play_state_.placement_point = temp_point;
}

void CoachNode::play_state_callback(rj_msgs::msg::PlayState::SharedPtr msg) {
    current_play_state_ = *msg;
    play_state_has_changed_ = true;
}

void CoachNode::world_state_callback(rj_msgs::msg::WorldState::SharedPtr msg) {
    // EDGE-CASE NOTE: If robots from both teams are bordering the ball possession will likely
    // switch repeatedly
    // TODO: subscribe to RoboStatuses in Radio to get has_ball_sense values
    if (!possessing_) {
        for (rj_msgs::msg::RobotState robotState : msg->our_robots) {
            // There definitely has to be a better way, but this works...
            if (rj_geometry::Point(robotState.pose.position.x, robotState.pose.position.y).dist_to(rj_geometry::Point(msg->ball.position.x, msg->ball.position.y)) < kRobotDiameter) {
                possessing_ = true;
                play_state_has_changed_ = true;
                return;
            }
        }
    } else {
        for (rj_msgs::msg::RobotState robotState : msg->their_robots) {
            // See above...
            if (rj_geometry::Point(robotState.pose.position.x, robotState.pose.position.y).dist_to(rj_geometry::Point(msg->ball.position.x, msg->ball.position.y)) < kRobotDiameter) {
                possessing_ = false;
                play_state_has_changed_ = true;
                return;
            }
        }
    }
}

void CoachNode::check_for_play_state_change() {
    if (play_state_has_changed_) {
        rj_msgs::msg::CoachStateInterpretation coach_message;

        switch (current_play_state_.restart) {
            case PlayState::Restart::Placement:
                coach_message.match_situation = MatchSituation::ball_placement;
                break;
            case PlayState::Restart::Kickoff:
                coach_message.match_situation = MatchSituation::kickoff;
                break;
            case PlayState::Restart::Direct:
            case PlayState::Restart::Indirect:
                coach_message.match_situation = MatchSituation::free_kick;
                break;
            case PlayState::Restart::Penalty:
                coach_message.match_situation = MatchSituation::penalty_kick;
                break;
        }

        if (current_play_state_.state == PlayState::State::Playing) {
            coach_message.match_situation = MatchSituation::in_play;
        }

        rj_msgs::msg::GlobalOverride override;

        switch (current_play_state_.state) {
            case PlayState::State::Halt:
                override.max_speed = 0;
                override.min_dist_from_ball = 0;
                break;
            case PlayState::State::Stop:
                override.max_speed = 1.5;
                override.min_dist_from_ball = 0.5;
                break;
            case PlayState::State::Playing:
                override.max_speed = -1;
                override.min_dist_from_ball = 0;
        }

        // publish new necessary information
        coach_message.override = override;

        coach_message.our_possession = possessing_;

        coach_pub_->publish(coach_message);

        play_state_has_changed_ = false;
    }
}