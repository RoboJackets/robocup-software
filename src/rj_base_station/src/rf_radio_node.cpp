#include "rj_base_station/rf_radio_node.hpp"

namespace base_station {

RFRadioNode::RFRadioNode() : rclcpp::Node("radio") {
    // Declare and parse parameters for the radio node
    declare_parameter<std::vector<int>>("robots", {0, 1, 2});
    declare_parameter<int>("hz", 100);
    declare_parameter<int>("csn", 0);
    declare_parameter<int>("ce", 0);
    declare_parameter<int>("irq", 0);
    declare_parameter<std::string>("pa", "low");
    declare_parameter<int>("channel", 0);
    
    transmit_robots_ = get_parameter("robots").as_integer_array();
    int64_t transmit_hz = get_parameter("hz").as_int();

    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        // NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const std::shared_ptr<rj_msgs::msg::TeamColor> color) {
            if (color->is_blue != blue_team_) {
                blue_team_ = color->is_blue;
            }
        }
    );

    // Create the publishers and subscriptions for each robot
    for (const int64_t& robot_id : transmit_robots_ ) {
        motion_subs_[robot_id] = create_subscription<rj_msgs::msg::MotionSetpoint>(
            control::topics::motion_setpoint_topic(static_cast<int>(robot_id)), rclcpp::QoS(1),
            // NOLINTNEXTLINE(performance-unnecessary-value-param)
            [this, robot_id](const std::shared_ptr<rj_msgs::msg::MotionSetpoint> msg) {
                motion_setpoints_[robot_id] = msg;
            }
        );
        motion_setpoints_[robot_id] = {};

        manipulator_subs_[robot_id] = create_subscription<rj_msgs::msg::ManipulatorSetpoint>(
            control::topics::manipulator_setpoint_topic(static_cast<int>(robot_id)), rclcpp::QoS(1),
            // NOLINTNEXTLINE(performance-unnecessary-value-param)
            [this, robot_id](const std::shared_ptr<rj_msgs::msg::ManipulatorSetpoint> msg) {
                manipulator_setpoints_[robot_id] = msg;
            }
        );
        manipulator_setpoints_[robot_id] = {};

        robot_status_pubs_[robot_id] =
            create_publisher<rj_msgs::msg::RobotStatus>(radio::topics::robot_status_topic(static_cast<int>(robot_id)), rclcpp::QoS(1));
        robot_status_queue_[robot_id] = std::nullopt;
    }

    // Bind the wall timer to send robot commands
    transmit_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / transmit_hz), [this]{
        publish_robot_statuses();
        send_motion_commands();
    });

    // Bind the callback handle
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) -> rcl_interfaces::msg::SetParametersResult
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = parse_parameters(params);

            return result;
        }
    );

    uint16_t csn = static_cast<uint16_t>(get_parameter("csn").as_int());
    uint16_t chip_enable = static_cast<uint16_t>(get_parameter("ce").as_int());
    radio_ = RF24(chip_enable, csn);
    if (!radio_.begin()) {
        throw std::runtime_error("Unable to Start Radio");
    }

    radio_.setPALevel(decode_pa(get_parameter("pa").as_string()));
    radio_.maskIRQ(true, true, false);
    radio_.setRetries(0, 0);
    radio_.setAutoAck(false);
    radio_.setCRCLength(RF24_CRC_8);
    radio_.setPayloadSize(rtp::ControlMessage::kSize);
    radio_.setChannel(static_cast<uint8_t>(get_parameter("channel").as_int()));
    radio_.setDataRate(RF24_2MBPS);
    radio_.stopListening();
    radio_.openReadingPipe(1, rtp::kBaseStationAddresses[static_cast<size_t>(blue_team_)].data());
;
    // Bind the GPIO Interrupt to the radio_gpio_callback routine
    irq_ = static_cast<uint8_t>(get_parameter("irq").as_int());
    gpioSetMode(irq_, PI_INPUT);
    gpioSetPullUpDown(irq_, PI_PUD_UP);
    gpioSetAlertFuncEx(irq_, &RFRadioNode::gpio_cb, this);
}

void RFRadioNode::send_motion_commands() {
    radio_.stopListening();
    radio_.setPayloadSize(rtp::ControlMessage::kSize);
    for (int64_t robot_id : transmit_robots_) {
        radio_.openWritingPipe(rtp::kRobotAddresses[static_cast<size_t>(blue_team_)][static_cast<size_t>(robot_id)].data());
        rtp::ControlMessage message = rtp::ControlMessage::from_ros(
            static_cast<unsigned int>(robot_id),
            blue_team_,
            motion_setpoints_[robot_id],
            manipulator_setpoints_[robot_id]
        );
        radio_.write(&message, rtp::ControlMessage::kSize);
    }
    radio_.openReadingPipe(1, rtp::kBaseStationAddresses[static_cast<size_t>(blue_team_)].data());
    radio_.startListening();
}

void RFRadioNode::publish_robot_statuses() {
    for (int64_t robot_id : transmit_robots_) {
        auto pair = robot_status_queue_.find(robot_id);
        if (pair != robot_status_queue_.end()) {
            rj_msgs::msg::RobotStatus robot_status = pair->second->to_ros();
            robot_status_pubs_[pair->second->robot_id]->publish(robot_status);
        }
    }
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void RFRadioNode::gpio_cb([[maybe_unused]] int gpio, int level, [[maybe_unused]] uint32_t tick, void * user) {
    if (level == FALLING_EDGE) {
        static_cast<RFRadioNode*>(user)->radio_gpio_callback();
    }
}

void RFRadioNode::radio_gpio_callback() {
    radio_.clearStatusFlags(RF24_IRQ_ALL);
    uint8_t pipe = 0;
    while (radio_.available(&pipe)) {
        uint8_t bytes = radio_.getPayloadSize();
        if (bytes >= 3) {
            rtp::RobotStatusMessage status = {};
            radio_.read(&status, bytes);
            robot_status_queue_[status.robot_id] = status;
        }
    }
    radio_.clearStatusFlags(RF24_IRQ_ALL);
}

rf24_pa_dbm_e RFRadioNode::decode_pa(const std::string& str) {
    if (str == "min") {
        return RF24_PA_MIN;
    }
    
    if (str == "low") {
        return RF24_PA_LOW;
    }
    
    if (str == "high") {
        return RF24_PA_HIGH;
    }
    
    if (str == "max") {
        return RF24_PA_MAX;
    }

    return RF24_PA_ERROR;
}

bool RFRadioNode::parse_parameters(const std::vector<rclcpp::Parameter>& params) {
    return std::all_of(params.begin(), params.end(), [this](const auto& param) {
        if (param.get_name() == "robots") {
            // Update publishers and subscribers
            std::unordered_set<int64_t> current_ids;
            current_ids.insert(transmit_robots_.begin(), transmit_robots_.end());

            std::vector<int64_t> new_transmit_robots = get_parameter("robots").as_integer_array();
            std::unordered_set<int64_t> new_ids;
            new_ids.insert(new_transmit_robots.begin(), new_transmit_robots.end());

            std::unordered_set<int64_t> remove_ids = current_ids;
            current_ids.erase(new_ids.begin(), new_ids.end());

            for (int64_t robot_id : remove_ids) {
                motion_subs_.erase(robot_id);
                motion_setpoints_.erase(robot_id);
                manipulator_subs_.erase(robot_id);
                manipulator_setpoints_.erase(robot_id);
                robot_status_pubs_.erase(robot_id);
                robot_status_queue_.erase(robot_id);
            }

            std::unordered_set<int64_t> add_ids = new_ids;
            add_ids.erase(current_ids.begin(), current_ids.end());

            for (int64_t robot_id : add_ids) {
                motion_subs_[robot_id] = create_subscription<rj_msgs::msg::MotionSetpoint>(
                    control::topics::motion_setpoint_topic(static_cast<int>(robot_id)), rclcpp::QoS(1),
                    // NOLINTNEXTLINE(performance-unnecessary-value-param)
                    [this, robot_id](const std::shared_ptr<rj_msgs::msg::MotionSetpoint> msg) {
                        motion_setpoints_[robot_id] = msg;
                    }
                );
                motion_setpoints_[robot_id] = {};

                manipulator_subs_[robot_id] = create_subscription<rj_msgs::msg::ManipulatorSetpoint>(
                    control::topics::manipulator_setpoint_topic(static_cast<int>(robot_id)), rclcpp::QoS(1),
                    // NOLINTNEXTLINE(performance-unnecessary-value-param)
                    [this, robot_id](const std::shared_ptr<rj_msgs::msg::ManipulatorSetpoint> msg) {
                        manipulator_setpoints_[robot_id] = msg;
                    }
                );
                manipulator_setpoints_[robot_id] = {};

                robot_status_pubs_[robot_id] = 
                    create_publisher<rj_msgs::msg::RobotStatus>(radio::topics::robot_status_topic(static_cast<int>(robot_id)), rclcpp::QoS(1));
                robot_status_queue_[robot_id] = std::nullopt;
            }

        } else if (param.get_name() == "hz") {
            int64_t new_transmit_hz = get_parameter("hz").as_int();
            transmit_timer_->cancel();
            transmit_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / new_transmit_hz), [this]{
                publish_robot_statuses();
                send_motion_commands();
            });
        } else if (param.get_name() == "pa") {
            rf24_pa_dbm_e power_level = decode_pa(get_parameter("pa").as_string());
            if (power_level == RF24_PA_ERROR) {
                return false;
            }
            radio_.setPALevel(power_level);
        } else if (param.get_name() == "channel") {
            radio_.setChannel(static_cast<uint8_t>(get_parameter("channel").as_int()));   
        } else if (param.get_name() == "id" || param.get_name() == "ce" || param.get_name() == "csn" || param.get_name() == "irq") {
            return false;
        }
        return true;
    });
}

} // namespace base_station

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (gpioInitialise() < 0) {
        std::cerr << "Unable to Initialize GPIO" << std::endl;
        return 1;
    }

    std::shared_ptr<base_station::RFRadioNode> node;
    try {
        node = std::make_shared<base_station::RFRadioNode>();
    } catch (const std::runtime_error& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}