#include "coach_node.hpp"

namespace strategy {
CoachNode::CoachNode(const rclcpp::NodeOptions& options) : Node("coach_node", options) {
    coach_state_pub_ =
        this->create_publisher<rj_msgs::msg::CoachState>("/strategy/coach_state", 10);
    coach_action_callback_timer_ = this->create_wall_timer(100ms, [this]() { coach_ticker(); });

    def_area_obstacles_pub_ =
        this->create_publisher<rj_geometry_msgs::msg::ShapeSet>("planning/def_area_obstacles", 10);

    global_obstacles_pub_ = this->create_publisher<rj_geometry_msgs::msg::ShapeSet>("planning/global_obstacles", 10);

    play_state_change_timer_ =
        this->create_wall_timer(100ms, [this]() { check_for_play_state_change(); });

    play_state_sub_ = this->create_subscription<rj_msgs::msg::PlayState>(
        "/referee/play_state", 10,
        [this](const rj_msgs::msg::PlayState::SharedPtr msg) { play_state_callback(msg); });

    positions_pub_ =
        this->create_publisher<rj_msgs::msg::PositionAssignment>("/strategy/positions", 10);

    world_state_sub_ = this->create_subscription<rj_msgs::msg::WorldState>(
        "/vision_filter/world_state", 10,
        [this](const rj_msgs::msg::WorldState::SharedPtr msg) { world_state_callback(msg); });

    // TODO: (https://app.clickup.com/t/867796fh2)sub to acknowledgement topic from AC
    // save state of acknowledgements, only spam until some long time has passed, or ack received
    /* ack_array[msg->ID] = true; */

    field_dimensions_sub_ = this->create_subscription<rj_msgs::msg::FieldDimensions>(
        "config/field_dimensions", 10, [this](const rj_msgs::msg::FieldDimensions::SharedPtr msg) {
            field_dimensions_callback(msg);
        });

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
                return;
            }
        }
    } else {
        possessing_ = false;
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

void CoachNode::coach_ticker() {
    assign_positions();
    check_for_play_state_change();
}

void CoachNode::check_for_play_state_change() {
    if (play_state_has_changed_) {
        rj_msgs::msg::CoachState coach_message;

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

        rj_msgs::msg::GlobalOverride global_override;

        switch (current_play_state_.state) {
            case PlayState::State::Halt:
                global_override.max_speed = 0;
                global_override.min_dist_from_ball = 0;
                break;
            case PlayState::State::Stop:
                global_override.max_speed = 1.5;
                global_override.min_dist_from_ball = 0.5;
                break;
            case PlayState::State::Playing:
                global_override.max_speed = -1;
                global_override.min_dist_from_ball = 0;
        }

        // publish new necessary information
        coach_message.global_override = global_override;

        coach_message.our_possession = possessing_;

        coach_state_pub_->publish(coach_message);

        play_state_has_changed_ = false;
    }
}

void CoachNode::assign_positions() {
    rj_msgs::msg::PositionAssignment positions_message;
    std::array<uint32_t, kNumShells> positions{};
    positions[0] = Positions::Goalie;
    if (!possessing_) {
        positions[1] = Positions::Offense;
        for (int i = 2; i < kNumShells; i++) {
            positions[i] = Positions::Defense;
        }
    } else {
        positions[1] = Positions::Defense;
        for (int i = 2; i < kNumShells; i++) {
            positions[i] = Positions::Offense;
        }
    }
    positions_message.client_positions = positions;
    positions_pub_->publish(positions_message);
}

void CoachNode::field_dimensions_callback(const rj_msgs::msg::FieldDimensions::SharedPtr& msg) {
    // Only publish once
    if (!field_updated_) {
        // publish defense areas as rectangular area obstacles
        /* From the old gameplay node: "The defense area, per the rules, is the box
         in front of each goal where only that team's goalie can be in and touch the ball." */

        SPDLOG_INFO("Updating field with message");
        SPDLOG_INFO("Message penalty dist {}", msg->penalty_long_dist);

        rj_geometry_msgs::msg::ShapeSet def_area_obstacles;

        rj_geometry_msgs::msg::Rect our_defense_area;

        rj_geometry_msgs::msg::Point top_left;
        top_left.x = msg->penalty_long_dist / 2 + msg->line_width;
        top_left.y = 0.0;

        rj_geometry_msgs::msg::Point bot_right;
        bot_right.x = msg->penalty_long_dist / 2 - msg->line_width;
        bot_right.y = msg->penalty_long_dist;

        std::array<rj_geometry_msgs::msg::Point, 2> our_def_area_pts = {top_left, bot_right};
        our_defense_area.pt = our_def_area_pts;

        std::vector<rj_geometry_msgs::msg::Rect> rectangles;
        rectangles.emplace_back(our_defense_area);

        def_area_obstacles.rectangles = rectangles;

        SPDLOG_INFO("made it to publishing obstacles");

        def_area_obstacles_pub_->publish(def_area_obstacles);

        // Only publish once
        field_updated_ = true;
    }
}

}  // namespace strategy
