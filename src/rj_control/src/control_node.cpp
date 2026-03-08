#include "rj_control/control_node.hpp"

namespace control {

ControlNode::ControlNode()
    : rclcpp::Node("control"),
      robot_id_(rj_utils::parse_id_from_namespace(get_name()))
{
    // Declare and parse parameters for the node
    declare_parameter<int>("hz", 30);
    declare_parameter<double>("barrier.gain", 100.0);
    int64_t tick_rate = get_parameter("hz").as_int();

    declare_parameter<float>("pid.translational.p.gain", 1.0);
    declare_parameter<float>("pid.translational.i.gain", 0.0);
    declare_parameter<float>("pid.translational.d.gain", 0.0);
    declare_parameter<float>("pid.translational.max_output", 3.5);
    declare_parameter<float>("pid.translational.p.limit", -1.0);
    declare_parameter<float>("pid.translational.i.limit", -1.0);
    declare_parameter<float>("pid.translational.d.limit", -1.0);
    declare_parameter<float>("pid.rotational.p.gain", 1.0);
    declare_parameter<float>("pid.rotational.i.gain", 0.0);
    declare_parameter<float>("pid.rotational.d.gain", 0.0);
    declare_parameter<float>("pid.rotational.max_output", 2 * M_PI);
    declare_parameter<float>("pid.rotational.p.limit", -1.0);
    declare_parameter<float>("pid.rotational.i.limit", -1.0);
    declare_parameter<float>("pid.rotational.d.limit", -1.0);

    barrier_certificate_ = std::make_unique<LinearCBF>(robot_id_, 100.0, 1.0 / 2.0, false);

    drawer_ = std::optional(rj_drawing::RosDebugDrawer(
        create_publisher<rj_drawing_msgs::msg::DebugDraw>(viz::topics::kDebugDrawTopic, 10),
        fmt::format("motion_control/{}", std::to_string(robot_id_))
    ));

    setpoint_pub_ = create_publisher<ControlCommand::Msg>(
        "control", rclcpp::QoS(1));
    world_state_sub_ = create_subscription<WorldState::Msg>(
        vision_filter::topics::kWorldStateTopic,
        rclcpp::QoS(1),
        // NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const WorldState::Msg::SharedPtr world_state_msg) {
            world_state_ = rj_convert::convert_from_ros(*world_state_msg);
        }
    );
    play_state_sub_ = create_subscription<PlayState::Msg>(
        referee::topics::kPlayStateTopic,
        rclcpp::QoS(1).transient_local(),
        // NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const PlayState::Msg::SharedPtr play_state_msg) {
            play_state_ = rj_convert::convert_from_ros(*play_state_msg);
        }
    );
    action_complete_pub_ = create_publisher<std_msgs::msg::Bool>(
        "action/complete", rclcpp::QoS(1).transient_local()
    );
    action_sub_ = create_subscription<action::Action::Msg>(
        "action", rclcpp::QoS(1).transient_local(),
        //NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const action::Action::Msg::SharedPtr action_msg) {
            action::Action new_action = rj_convert::convert_from_ros(*action_msg);
            set_action(new_action);
        }
    );
    field_dimensions_sub_ = create_subscription<FieldDimensions::Msg>(
        config_server::topics::kFieldDimensionsTopic, rclcpp::QoS(1).transient_local(),
        // NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const FieldDimensions::Msg::SharedPtr field_dimensions) {
            field_dimensions_ = rj_convert::convert_from_ros(*field_dimensions);
        }
    );
    goalie_sub_ = create_subscription<rj_msgs::msg::Goalie>(
        referee::topics::kGoalieTopic, rclcpp::QoS(1).transient_local(),
        // NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const rj_msgs::msg::Goalie::SharedPtr goalie_msg) {
            if (goalie_msg->goalie_id == robot_id_) {
                barrier_certificate_->set_goalie(true);
            } else {
                barrier_certificate_->set_goalie(false);
            }
        }
    );

    control_update_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / tick_rate),
        [this]() {
            run();
        }
    );

    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) -> rcl_interfaces::msg::SetParametersResult
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto& param : params) {
                if (param.get_name() == "hz") {
                    int64_t tick_rate = param.as_int();
                    control_update_timer_->cancel();
                    control_update_timer_ = create_wall_timer(
                        std::chrono::milliseconds(1000 / tick_rate),
                        [this]() {
                            run();
                        }
                    );
                    barrier_certificate_->set_dt(1.0 / static_cast<double>(tick_rate));
                } else if (param.get_name() == "barrier.gain") {
                    double barrier_gain = param.as_double();
                    barrier_certificate_->set_control_gain(barrier_gain);
                }
            }

            return result;
        }
    );
}

void ControlNode::run() {
    ControlCommand control_command;
    bool avoid_ball = false;
    if (!play_state_.is_halt() && action_.has_value()) {
        auto controller = controllers_.find(action_.value().get_type());
        if (world_state_.has_value() && controller != controllers_.end()) {
            control_command = controller->second->update(world_state_.value(), field_dimensions_, action_.value());

            // Publish if the current action has been completed
            bool complete = controller->second->complete(world_state_.value(), field_dimensions_, action_.value());
            if (complete) {
                complete_frames_ += 1;
            } else {
                complete_frames_ = 0;
            }

            if (complete_frames_ > 30 && !completed_) {
                std_msgs::msg::Bool msg;
                msg.data = true;
                action_complete_pub_->publish(msg);
                completed_ = true;
            } else if (complete_frames_ <= 30 && completed_) {
                std_msgs::msg::Bool msg;
                msg.data = false;
                action_complete_pub_->publish(msg);
                completed_ = false;
            }

            avoid_ball = controller->second->avoid_ball(world_state_.value(), field_dimensions_, action_.value());
        } else {
            // Don't move
            control_command = {};
        }

        rj_geometry::Twist barrier_velocity = barrier_certificate_->apply(
            world_state_.value(),
            play_state_,
            field_dimensions_,
            control_command.velocity(),
            avoid_ball
        );

        // control_command.set_velocity(
        //     barrier_certificate_->apply(
        //         world_state_.value(),
        //         play_state_,
        //         field_dimensions_,
        //         control_command.velocity(),
        //         avoid_ball
        //     )
        // );
        } else {
        control_command = {};
    }

    // Convert setpoint from global coordinates to robot coordinates
    if (world_state_.has_value()) {
        auto robot_pose = world_state_->get_robot(true, robot_id_).pose;
        rj_geometry::Twist body_velocity(control_command.velocity().linear().rotated(M_PI_2 - robot_pose.heading()),
            control_command.velocity().angular());
        control_command.set_velocity(body_velocity);
    }
    setpoint_pub_->publish(rj_convert::convert_to_ros(control_command));
    
    // Debug Drawing
    if (drawer_.has_value() && world_state_.has_value()) {
        auto robot_pose = world_state_->get_robot(true, robot_id_).pose;
        // Draw Robot Position
        drawer_.value().draw_circle(
            rj_geometry::Circle(robot_pose.position(), 0.15),
            QColor(0, 255, 0, 0)
        );

        // Draw Target Position
        // std::optional<planning::RobotInstant> desired_position = trajectory_->evaluate(RJ::now());
        // if (!desired_position.has_value()) {
        //     desired_position = trajectory_->last();
        // }
        // if (desired_position.has_value()) {
        //     drawer_.value().draw_circle(
        //         rj_geometry::Circle(desired_position.value().pose.position(), 0.15),
        //         QColor(0, 255, 0, 0)
        //     );
        // }

        // Draw Velocity Line
        drawer_.value().draw_segment(
            rj_geometry::Segment(robot_pose.position(), robot_pose.position() - control_command.velocity().linear()),
            Qt::blue
        );

        // Publish Debug Drawing
        drawer_.value().publish();
    }
}

void ControlNode::create_controllers() {
    auto position_controller = std::make_unique<PositionController>(robot_id_, shared_from_this());
    controllers_[position_controller->id()] = std::move(position_controller);
    auto pose_controller = std::make_unique<PoseController>(robot_id_, shared_from_this());
    controllers_[pose_controller->id()] = std::move(pose_controller);
    auto shoot_controller = std::make_unique<ShootController>(robot_id_, shared_from_this());
    controllers_[shoot_controller->id()] = std::move(shoot_controller);
    auto collect_controller = std::make_unique<CollectController>(robot_id_, shared_from_this());
    controllers_[collect_controller->id()] = std::move(collect_controller);
    auto pass_controller = std::make_unique<PassController>(robot_id_, shared_from_this());
    controllers_[pass_controller->id()] = std::move(pass_controller);
    auto dribble_controller = std::make_unique<DribbleController>(robot_id_, shared_from_this());
    controllers_[dribble_controller->id()] =std::move(dribble_controller);
    auto clear_controller = std::make_unique<ClearController>(robot_id_, shared_from_this());
    controllers_[clear_controller->id()] = std::move(clear_controller);
    auto mark_robot_controller = std::make_unique<MarkRobotController>(robot_id_, shared_from_this());
    controllers_[mark_robot_controller->id()] = std::move(mark_robot_controller);
    auto rotate_controller = std::make_unique<RotateController>(robot_id_, shared_from_this());
    controllers_[rotate_controller->id()] = std::move(rotate_controller);
}

void ControlNode::set_action(action::Action new_action) {
    if (world_state_.has_value()) {
        // Stop old controller
        if (action_.has_value()) {
            auto old_controller = controllers_.find(action_->get_type());
            if (old_controller != controllers_.end()) {
                old_controller->second->stop(world_state_.value(), field_dimensions_, action_.value());
            }
        }

        // Start new controller
        auto new_controller = controllers_.find(new_action.get_type());
        if (new_controller != controllers_.end()) {
            new_controller->second->start(world_state_.value(), field_dimensions_, new_action);
        }

        action_ = new_action;
        completed_ = false;
    } else {
        completed_ = true;
    }

    // Publish the state of the new action
    std_msgs::msg::Bool msg;
    msg.data = completed_;
    action_complete_pub_->publish(msg);
}

} // namespace control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto node = std::make_shared<control::ControlNode>();
    node->create_controllers();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}