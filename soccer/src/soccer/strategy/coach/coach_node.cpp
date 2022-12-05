#include "coach_node.hpp"

CoachNode::CoachNode(const rclcpp::NodeOptions& options) : Node("coach_node", options) {
    coach_state_pub_ =
        this->create_publisher<rj_msgs::msg::CoachStateInterpretation>("/strategy/coach_state", 10);
    play_state_change_timer_ =
        this->create_wall_timer(100ms, [this]() { check_for_play_state_change(); });

    play_state_sub_ = this->create_subscription<rj_msgs::msg::PlayState>(
        "/referee/play_state", 10,
        [this](const rj_msgs::msg::PlayState::SharedPtr msg) { play_state_callback(msg); });

    world_state_sub_ = this->create_subscription<rj_msgs::msg::WorldState>(
        "/vision_filter/world_state", 10,
        [this](const rj_msgs::msg::WorldState::SharedPtr msg) { world_state_callback(msg); });

    // initialize all of the robot status subscriptions
    for (size_t i = 0; i < kNumShells; i++) {
        robot_status_subs_[i] = this->create_subscription<rj_msgs::msg::RobotStatus>(
            fmt::format("/radio/robot_status/robot_{}", i), 10,
            [this](const rj_msgs::msg::RobotStatus::SharedPtr msg) {
                ball_sense_callback(msg, true);
            });
    }

    current_play_state_.state = PlayState::State::Halt;
    current_play_state_.restart = PlayState::Restart::Kickoff;
    current_play_state_.our_restart = true;

    rj_geometry_msgs::msg::Point temp_point;
    temp_point.x = -1;
    temp_point.y = -1;
    current_play_state_.placement_point = temp_point;
}

void CoachNode::play_state_callback(const rj_msgs::msg::PlayState::SharedPtr msg) {
    current_play_state_ = *msg;
    play_state_has_changed_ = true;
}

void CoachNode::world_state_callback(const rj_msgs::msg::WorldState::SharedPtr msg) {
    // EDGE-CASE NOTE: If robots from both teams are bordering the ball possession will likely
    // TODO: (https://app.clickup.com/t/31w0jay)
    // switch repeatedly
    if (!possessing_) {
        for (rj_msgs::msg::RobotState robot_state : msg->our_robots) {
            if (rj_geometry::Point::nearly_equals(
                    rj_convert::convert_from_ros(robot_state.pose.position),
                    rj_convert::convert_from_ros(msg->ball.position), kRobotDiameter)) {
                possessing_ = true;
                play_state_has_changed_ = true;
                return;
            }
        }
    } else {
        for (rj_msgs::msg::RobotState robot_state : msg->their_robots) {
            if (rj_geometry::Point::nearly_equals(
                    rj_convert::convert_from_ros(robot_state.pose.position),
                    rj_convert::convert_from_ros(msg->ball.position), kRobotDiameter)) {
                possessing_ = false;
                play_state_has_changed_ = true;
                return;
            }
        }
    }
}

void CoachNode::ball_sense_callback(const rj_msgs::msg::RobotStatus::SharedPtr msg, bool our_team) {
    if (our_team && !possessing_) {
        if (msg->has_ball_sense) {
            possessing_ = true;
            play_state_has_changed_ = true;
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

        coach_state_pub_->publish(coach_message);

        play_state_has_changed_ = false;
    }
}