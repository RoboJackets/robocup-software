#include "coach_node.hpp"

#include "rj_constants/topic_names.hpp"

namespace strategy {
CoachNode::CoachNode(const rclcpp::NodeOptions& options) : Node("coach_node", options) {
    coach_state_pub_ =
        this->create_publisher<rj_msgs::msg::CoachState>(topics::kCoachStateTopic, 10);
    coach_action_callback_timer_ = this->create_wall_timer(100ms, [this]() { coach_ticker(); });

    def_area_obstacles_pub_ = this->create_publisher<rj_geometry_msgs::msg::ShapeSet>(
        ::planning::topics::kDefAreaObstaclesTopic, 10);

    global_obstacles_pub_ = this->create_publisher<rj_geometry_msgs::msg::ShapeSet>(
        ::planning::topics::kGlobalObstaclesTopic, 10);

    play_state_sub_ = this->create_subscription<rj_msgs::msg::PlayState>(
        ::referee::topics::kPlayStateTopic, 10,
        [this](const rj_msgs::msg::PlayState::SharedPtr msg) { play_state_callback(msg); });

    positions_pub_ =
        this->create_publisher<rj_msgs::msg::PositionAssignment>(topics::kPositionsTopic, 10);

    overrides_sub_ = this->create_subscription<rj_msgs::msg::PositionAssignment>(
        "/strategy/position_overrides", 10,
        [this](const rj_msgs::msg::PositionAssignment::SharedPtr msg) { overrides_callback(msg); });
    world_state_sub_ = this->create_subscription<rj_msgs::msg::WorldState>(
        ::vision_filter::topics::kWorldStateTopic, 10,
        [this](const rj_msgs::msg::WorldState::SharedPtr msg) { world_state_callback(msg); });

    goalie_sub_ = this->create_subscription<rj_msgs::msg::Goalie>(
        ::referee::topics::kGoalieTopic, 10,
        [this](const rj_msgs::msg::Goalie::SharedPtr msg) { goalie_callback(msg); });

    // TODO: (https://app.clickup.com/t/867796fh2)sub to acknowledgement topic from AC
    // save state of acknowledgements, only spam until some long time has passed, or ack
    // received
    /* ack_array[msg->ID] = true; */

    field_dimensions_sub_ = this->create_subscription<rj_msgs::msg::FieldDimensions>(
        ::config_server::topics::kFieldDimensionsTopic, 10,
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr msg) {
            field_dimensions_callback(msg);
        });

    // initialize all of the robot status subscriptions
    for (size_t i = 0; i < kNumShells; i++) {
        robot_status_subs_[i] = this->create_subscription<rj_msgs::msg::RobotStatus>(
            ::radio::topics::robot_status_topic(i), 10,
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
        coach_message.play_state = current_play_state_;
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
                // Unbounded speed. Setting to -1 or 0 crashes planner, so use large number
                // instead.
                global_override.max_speed = 10.0;
                global_override.min_dist_from_ball = 0;
                break;
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
    rj_msgs::msg::PositionAssignment positions_message;
    std::array<uint32_t, kNumShells> positions{};
    positions[goalie_id_] = Positions::Goalie;
    if (!possessing_) {
        // except goalie, all robots set to defense
        for (int i = 0; i < kNumShells; i++) {
            if (i != goalie_id_) {
                positions[i] = Positions::Defense;
            }
        }
    } else {
        // except goalie, all robots set to offense
        for (int i = 0; i < kNumShells; i++) {
            if (i != goalie_id_) {
                positions[i] = Positions::Offense;
            }
        }
    }

    // Check Overrides
    if (have_overrides_) {
        for (int i = 0; i < kNumShells; i++) {
            if (i != goalie_id_ && current_overrides_[i] == 1 || current_overrides_[i] == 2) {
                positions[i] = current_overrides_[i];
            }
        }
    }

    positions_message.client_positions = positions;
    positions_pub_->publish(positions_message);
}

void CoachNode::overrides_callback(const rj_msgs::msg::PositionAssignment::SharedPtr& msg) {
    current_overrides_ = msg->client_positions;
    have_overrides_ = true;
}

void CoachNode::field_dimensions_callback(const rj_msgs::msg::FieldDimensions::SharedPtr& msg) {
    current_field_dimensions_ = rj_convert::convert_from_ros(*msg);
    have_field_dimensions_ = true;
}

void CoachNode::goalie_callback(const rj_msgs::msg::Goalie::SharedPtr& msg) {
    goalie_id_ = msg->goalie_id;
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
    auto our_defense_area{std::make_shared<rj_geometry::Rect>(
        std::move(current_field_dimensions_.our_defense_area()))};

    // Sometimes there is a greater distance we need to keep:
    // https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_too_close_to_opponent_defense_area
    // TODO(sid-parikh): update this conditional. gameplay_node used different set of checks
    // than rules imply
    bool is_extra_dist_necessary = (current_play_state_.state == PlayState::State::Stop ||
                                    current_play_state_.restart == PlayState::Restart::Free);

    // Also add a slack around the box
    float slack_around_box{0.3f};

    auto their_defense_area =
        is_extra_dist_necessary
            ? std::make_shared<rj_geometry::Rect>(
                  std::move(current_field_dimensions_.their_defense_area_padded(slack_around_box)))
            : std::make_shared<rj_geometry::Rect>(
                  std::move(current_field_dimensions_.their_defense_area()));

    // Combine both defense areas into ShapeSet
    rj_geometry::ShapeSet def_area_obstacles{};
    def_area_obstacles.add(our_defense_area);
    def_area_obstacles.add(their_defense_area);

    return def_area_obstacles;
}

rj_geometry::ShapeSet CoachNode::create_goal_wall_obstacles() {
    // Create physical walls of the goals as static obstacles
    double physical_goal_board_width = 0.1;
    float goal_width = current_field_dimensions_.goal_width();
    float goal_depth = current_field_dimensions_.goal_depth();
    float field_length = current_field_dimensions_.length();

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
