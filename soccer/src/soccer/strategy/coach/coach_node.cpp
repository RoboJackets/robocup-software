#include "coach_node.hpp"

namespace strategy {
CoachNode::CoachNode(const rclcpp::NodeOptions& options) : Node("coach_node", options) {
    coach_state_pub_ =
        this->create_publisher<rj_msgs::msg::CoachState>("/strategy/coach_state", 10);

    coach_action_callback_timer_ = this->create_wall_timer(100ms, [this]() {    coach_ticker(); });

    def_area_obstacles_pub_ =
        this->create_publisher<rj_geometry_msgs::msg::ShapeSet>("planning/def_area_obstacles", 10);

    global_obstacles_pub_ =
        this->create_publisher<rj_geometry_msgs::msg::ShapeSet>("planning/global_obstacles", 10);

    play_state_sub_ = this->create_subscription<rj_msgs::msg::PlayState>(
        "/referee/play_state", 10,
        [this](const rj_msgs::msg::PlayState::SharedPtr msg) { play_state_callback(msg); });

    positions_pub_ =
        this->create_publisher<rj_msgs::msg::PositionAssignment>("/strategy/positions", 10);

    world_state_sub_ = this->create_subscription<rj_msgs::msg::WorldState>(
        "/vision_filter/world_state", 10,
        [this](const rj_msgs::msg::WorldState::SharedPtr msg) { world_state_callback(msg); });

    position_ack_sub_ = this->create_subscription<rj_msgs::msg::PositionAck>(
        "/strategy/position_ack", 10,
        [this](const rj_msgs::msg::PositionAck::SharedPtr msg) { position_ack_callback(msg); });

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
    if (!all_pos_acks) {
        assign_positions();
    }
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
                // Unbounded speed. Setting to -1 or 0 crashes planner, so use large number instead.
                global_override.max_speed = 10.0;
                global_override.min_dist_from_ball = 0;
        }

        // publish new necessary information
        coach_message.global_override = global_override;

        coach_message.our_possession = possessing_;

        coach_state_pub_->publish(coach_message);

        publish_static_obstacles();

        play_state_has_changed_ = false;
    }
}

void CoachNode::assign_positions() {
    all_pos_acks = false;
    
    rj_msgs::msg::PositionAssignment goalie_position_msg;
    goalie_position_msg.client_position = Positions::Goalie;
    generate_uid(goalie_position_msg);
    positions_pub_->publish(goalie_position_msg);
    if (!possessing_) {
        rj_msgs::msg::PositionAssignment offense_position_msg;
        offense_position_msg.client_position = Positions::Offense;
        generate_uid(offense_position_msg);
        positions_pub_->publish(offense_position_msg);
        for (int i = 2; i < kNumShells; i++) {
            rj_msgs::msg::PositionAssignment defense_position_msg;
            defense_position_msg.client_position = Positions::Defense;
            generate_uid(defense_position_msg);
            positions_pub_->publish(defense_position_msg);
        }
    } else {
        rj_msgs::msg::PositionAssignment defense_position_msg;
        defense_position_msg.client_position = Positions::Defense;
        generate_uid(defense_position_msg);
        positions_pub_->publish(defense_position_msg);
        for (int i = 2; i < kNumShells; i++) {
            rj_msgs::msg::PositionAssignment offense_position_msg;
            offense_position_msg.client_position = Positions::Offense;
            generate_uid(offense_position_msg);
            positions_pub_->publish(offense_position_msg);
        }
    }

    for (int i = 0; i < client_acknowledgements_.length; i++) {
        client_acknowledgements_[i] = 0;
    }

}

void CoachNode::position_ack_callback(const rj_msgs::msg::PositionAck::SharedPtr& msg) {
    if((goalie_position_msg.request_uid == msg->response_uid || defense_position_msg.request_uid == msg->response_uid || offense_position_msg.request_uid == msg->response_uid) && msg->acknowledgement == 1) {
        client_acknowledgements_[msg->robot_id] = 1;
    } else {
        client_acknowledgements_[msg->robot_id] = 0;
    }

    all_pos_acks = true;
}

//TODO: Create a function that checks if the first 6 bots have been acknowledged. It should return a boolean. Check that boolean in the coach ticker. 
void CoachNode::field_dimensions_callback(const rj_msgs::msg::FieldDimensions::SharedPtr& msg) {
    current_field_dimensions_ = *msg;
    have_field_dimensions_ = true;
}

void CoachNode::publish_static_obstacles() {
    if (have_field_dimensions_) {
        rj_geometry::ShapeSet defense_area_obstacles = create_defense_area_obstacles();
        rj_geometry::ShapeSet goal_wall_obstacles = create_goal_wall_obstacles();

        def_area_obstacles_pub_->publish(rj_convert::convert_to_ros(defense_area_obstacles));
        global_obstacles_pub_->publish(rj_convert::convert_to_ros(goal_wall_obstacles));
    }
}

rj_geometry::ShapeSet CoachNode::create_defense_area_obstacles() {
    // Create defense areas as rectangular area obstacles

    // Create our defense area using field dimensions
    float def_long_dist = current_field_dimensions_.penalty_long_dist / 2.0f;
    float def_short_dist = current_field_dimensions_.penalty_short_dist;
    float line_width = current_field_dimensions_.line_width;
    float field_length = current_field_dimensions_.length;

    rj_geometry::Point o_top_left{def_long_dist + line_width, 0.0};

    rj_geometry::Point o_bot_right{-def_long_dist - line_width, def_short_dist};

    auto our_defense_area{std::make_shared<rj_geometry::Rect>(o_top_left, o_bot_right)};

    // Create opponent defense area using field dimensions

    // Sometimes there is a greater distance we need to keep:
    // https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_too_close_to_opponent_defense_area
    // TODO(sid-parikh): update this conditional. gameplay_node used different set of checks
    // than rules imply
    bool is_extra_dist_necessary = (current_play_state_.state == PlayState::State::Stop ||
                                    current_play_state_.restart == PlayState::Restart::Direct ||
                                    current_play_state_.restart == PlayState::Restart::Indirect);

    // Also add a slack around the box
    double slack_around_box = 0.1;
    double extra_dist = is_extra_dist_necessary ? 0.2 + slack_around_box : 0;
    double left_x = def_long_dist + line_width + extra_dist;

    rj_geometry::Point t_bot_left{left_x, field_length};

    rj_geometry::Point t_top_right{-left_x,
                                   field_length - (def_short_dist + line_width + extra_dist)};

    auto their_defense_area{std::make_shared<rj_geometry::Rect>(t_bot_left, t_top_right)};

    // Combine both defense areas into ShapeSet
    rj_geometry::ShapeSet def_area_obstacles{};
    def_area_obstacles.add(our_defense_area);
    def_area_obstacles.add(their_defense_area);

    return def_area_obstacles;
}

rj_geometry::ShapeSet CoachNode::create_goal_wall_obstacles() {
    // Create physical walls of the goals as static obstacles
    double physical_goal_board_width = 0.1;
    float goal_width = current_field_dimensions_.goal_width;
    float goal_depth = current_field_dimensions_.goal_depth;
    float field_length = current_field_dimensions_.length;

    // Each goal is three rectangles
    rj_geometry::Point og1_1{goal_width / 2, -goal_depth};
    rj_geometry::Point og1_2{-goal_width / 2, -goal_depth - physical_goal_board_width};
    auto our_goal_1{std::make_shared<rj_geometry::Rect>(og1_1, og1_2)};

    rj_geometry::Point og2_1{goal_width / 2, -goal_depth};
    rj_geometry::Point og2_2{goal_width / 2 + physical_goal_board_width, 0.0};
    auto our_goal_2{std::make_shared<rj_geometry::Rect>(og2_1, og2_2)};

    rj_geometry::Point og3_1{-goal_width / 2, -goal_depth};
    rj_geometry::Point og3_2{-goal_width / 2 - physical_goal_board_width, 0.0};
    auto our_goal_3{std::make_shared<rj_geometry::Rect>(og3_1, og3_2)};

    rj_geometry::Point tg1_1{goal_width / 2, field_length + goal_depth};
    rj_geometry::Point tg1_2{-goal_width / 2,
                             field_length + goal_depth + physical_goal_board_width};
    auto their_goal_1{std::make_shared<rj_geometry::Rect>(tg1_1, tg1_2)};

    rj_geometry::Point tg2_1{goal_width / 2, field_length + goal_depth};
    rj_geometry::Point tg2_2{goal_width / 2 + physical_goal_board_width, field_length};
    auto their_goal_2{std::make_shared<rj_geometry::Rect>(tg2_1, tg2_2)};

    rj_geometry::Point tg3_1{-goal_width / 2, field_length + goal_depth};
    rj_geometry::Point tg3_2{-goal_width / 2 - physical_goal_board_width, field_length};
    auto their_goal_3{std::make_shared<rj_geometry::Rect>(tg3_1, tg3_2)};

    // Put all the walls into a ShapeSet and publish it
    rj_geometry::ShapeSet goal_wall_obstacles{};
    goal_wall_obstacles.add(our_goal_1);
    goal_wall_obstacles.add(our_goal_2);
    goal_wall_obstacles.add(our_goal_3);
    goal_wall_obstacles.add(their_goal_1);
    goal_wall_obstacles.add(their_goal_2);
    goal_wall_obstacles.add(their_goal_3);

    return goal_wall_obstacles;
}

}  // namespace strategy
