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

    alive_robots_sub_ = create_subscription<rj_msgs::msg::AliveRobots>(
        "strategy/alive_robots", 1,
        [this](rj_msgs::msg::AliveRobots::SharedPtr msg) { alive_robots_callback(msg); });

    game_settings_sub_ = create_subscription<rj_msgs::msg::GameSettings>(
        "config/game_settings", 1,
        [this](rj_msgs::msg::GameSettings::SharedPtr msg) { game_settings_callback(msg); });

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

void CoachNode::alive_robots_callback(const rj_msgs::msg::AliveRobots::SharedPtr& msg) {
    alive_robots_ = msg->alive_robots;
}

void CoachNode::game_settings_callback(const rj_msgs::msg::GameSettings::SharedPtr& msg) {
    is_simulated_ = msg->simulation;
}

void CoachNode::coach_ticker() {
    assign_positions();
    check_for_play_state_change();
}

void CoachNode::check_for_play_state_change() {
    if (play_state_has_changed_) {
        rj_msgs::msg::CoachState coach_message;
        coach_message.play_state = current_play_state_;

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

    switch (current_play_state_.restart) {
        case PlayState::Restart::Penalty:
            assign_positions_penalty(positions);
            break;
        case PlayState::Restart::Kickoff:
            assign_positions_kickoff(positions);
            break;
        case PlayState::Restart::Free:
            assign_positions_freekick(positions);
        case PlayState::Restart::Placement:
            // TODO: Placement Position Assignment
        case PlayState::Restart::None:
        default:
            // Normal Play
            assign_positions_normal(positions);
    }

    positions_message.client_positions = positions;
    positions_pub_->publish(positions_message);
}

void CoachNode::assign_positions_penalty(std::array<uint32_t, kNumShells>& positions) {
    for (size_t i = 0; i < kNumShells; i++) {
        if (i != goalie_id_) {
            // TODO: Update this position to Line
            positions[i] = Positions::Defense;
        }
    }

    // If our restart, then we need a robot to kick. Otherwise, all Line is fine
    if (current_play_state_.our_restart) {
        switch (current_play_state_.state) {
            case PlayState::State::Setup:
                // Lowest non-goalie robot set to PenaltyPlayer
                if (goalie_id_ == 0) {
                    positions[1] = Positions::PenaltyPlayer;
                } else {
                    positions[0] = Positions::PenaltyPlayer;
                }
                break;
            case PlayState::State::Ready:
            case PlayState::State::PenaltyPlaying:
                // Lowest non-goalie robot set to Goal Kicker
                if (goalie_id_ == 0) {
                    positions[1] = Positions::GoalKicker;
                } else {
                    positions[0] = Positions::GoalKicker;
                }
                break;
            default:
                SPDLOG_WARN("Invalid state for penalty restart");
                assign_positions_normal(positions);
        }
    }
}

void CoachNode::assign_positions_kickoff(std::array<uint32_t, kNumShells>& positions) {
    for (size_t i = 0; i < kNumShells; i++) {
        if (i != goalie_id_) {
            // Non-kicking robots play defense
            positions[i] = Positions::Defense;
        }
    }

    // If our restart, make one a kicker
    if (current_play_state_.our_restart) {
        switch (current_play_state_.state) {
            case PlayState::State::Setup:
                // Lowest non-goalie robot set to Penalty Player
                // TODO: Update this position to Kickoff Player?
                if (goalie_id_ == 0) {
                    positions[1] = Positions::PenaltyPlayer;
                } else {
                    positions[0] = Positions::PenaltyPlayer;
                }
                break;
            case PlayState::State::Ready:
                // Lowest non-goalie robot set to Goal Kicker
                if (goalie_id_ == 0) {
                    positions[1] = Positions::GoalKicker;
                } else {
                    positions[0] = Positions::GoalKicker;
                }
                break;
            default:
                SPDLOG_WARN("Invalid state for kickoff restart");
                assign_positions_normal(positions);
        }
    }
}

void CoachNode::assign_positions_freekick(std::array<uint32_t, kNumShells>& positions) {
    for (size_t i = 0; i < kNumShells; i++) {
        if (i != goalie_id_) {
            // Non-kicking robots play defense
            positions[i] = Positions::Defense;
        }
    }

    // If our restart, make one a kicker
    if (current_play_state_.our_restart) {
        switch (current_play_state_.state) {
            // Free Kick does not have setup state
            case PlayState::State::Ready:
                // Lowest non-goalie robot set to Goal Kicker
                if (goalie_id_ == 0) {
                    // Offense role should shoot. Placeholder for now.
                    positions[1] = Positions::Offense;
                } else {
                    positions[0] = Positions::Offense;
                }
                break;
            default:
                SPDLOG_WARN("Invalid state for free kick restart");
                assign_positions_normal(positions);
        }
    }
}

void CoachNode::assign_positions_normal(std::array<uint32_t, kNumShells>& positions) {
    // BEGIN COMP 2023 PATCH CODE

    // Assign Robots to positions in the following order:
    //  1. 1 Goalie
    //  2. 2 Offense
    //  3. Remaining Defense (always 1 defender)
    int assign_num = 0;
    for (u_int8_t robot_id = 0; robot_id < kNumShells; robot_id++) {
        if (check_robot_alive(robot_id)) {
            switch (assign_num) {
                case 0:
                    positions[robot_id] = Positions::Goalie;
                    break;
                case 1:
                    positions[robot_id] = Positions::Offense;
                    break;
                case 2:
                    positions[robot_id] = Positions::Defense;
                    break;
                case 3:
                    positions[robot_id] = Positions::Offense;
                    break;
                default:
                    positions[robot_id] = Positions::Defense;
                    break;
            }
            assign_num++;
        }
    }

    // END COMP 2023 PATCH CODE

    // if (!possessing_) {
    //     // All robots set to defense
    //     for (int i = 0; i < kNumShells; i++) {
    //         if (i != goalie_id_) {
    //             positions[i] = Positions::Defense;
    //         }
    //     }
    //     // Lowest non-goalie robot set to offense
    //     if (goalie_id_ == 0) {
    //         positions[1] = Positions::Offense;
    //     } else {
    //         positions[0] = Positions::Offense;
    //     }
    // } else {
    //     // All robots set to offense
    //     for (int i = 0; i < kNumShells; i++) {
    //         if (i != goalie_id_) {
    //             positions[i] = Positions::Offense;
    //         }
    //     }
    //     // Lowest non-goalie robot set to defense
    //     if (goalie_id_ == 0) {
    //         positions[1] = Positions::Defense;
    //     } else {
    //         positions[0] = Positions::Defense;
    //     }
    // }

    // // Check Overrides
    // if (have_overrides_) {
    //     for (int i = 0; i < kNumShells; i++) {
    //         if (i != goalie_id_ && current_overrides_[i] == 1 || current_overrides_[i] == 2) {
    //             positions[i] = current_overrides_[i];
    //         }
    //     }
    // }
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
    // Put all the walls into a ShapeSet and publish it
    rj_geometry::ShapeSet goal_wall_obstacles{};
    goal_wall_obstacles.add(current_field_dimensions_.our_goal_walls());
    goal_wall_obstacles.add(current_field_dimensions_.their_goal_walls());

    return goal_wall_obstacles;
}

bool CoachNode::check_robot_alive(u_int8_t robot_id) {
    if (!is_simulated_) {
        return std::find(alive_robots_.begin(), alive_robots_.end(), robot_id) !=
               alive_robots_.end();
    } else {
        // TODO (Nathaniel): In the future store a world state reference and check the robots
        // location to determine if the robot is alive
        return true;
    }
}

}  // namespace strategy
