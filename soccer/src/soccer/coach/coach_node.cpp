#include "coach_node.hpp"

using std::placeholders::_1;

CoachNode::CoachNode(const rclcpp::NodeOptions& options) : Node("coach_node", options) {
    coach_pub_ = this->create_publisher<rj_msgs::msg::Coach>("/strategy/coach", 10);
    playstate_change_timer_ = this->create_wall_timer(
        100ms, [this]() {
        check_for_playstate_change();   }));

    playstate_sub_ = this->create_subscription<rj_msgs::msg::PlayState>(
        "/referee/play_state", 10,
        [this](rj_msgs::msg::PlayState::SharedPtr msg) { playstate_callback(msg); });

    current_play_state_.state = PlayState::State::Halt;
    current_play_state_.restart = PlayState::Restart::Kickoff;
    current_play_state_.our_restart = true;

    rj_geometry_msgs::msg::Point temp_point;
    temp_point.x = -1;
    temp_point.y = -1;
    current_play_state_.placement_point = temp_point;
}

void CoachNode::playstate_callback(const rj_msgs::msg::PlayState::SharedPtr& msg) {
    current_play_state_ = *msg;
    playstate_has_changed_ = true;
}

void CoachNode::check_for_playstate_change() {
    if (playstate_has_changed_) {
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

        // TODO: get possession from wherever possession is from

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

        coach_pub_->publish(coach_message);

        playstate_has_changed_ = false;
    }
}