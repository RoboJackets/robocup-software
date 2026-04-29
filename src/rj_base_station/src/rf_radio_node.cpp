#include "rj_base_station/rf_radio_node.hpp"

#include <rj_msgs/msg/detail/motion_setpoint__struct.hpp>

namespace base_station {

RFRadioNode::RFRadioNode() : Node("radio"), running_(true) {
    // Declare parameters and parse parameters for the radio node
    declare_parameter<std::vector<int>>("robots", {0, 1, 2});
    declare_parameter<int>("hz", 100);
    declare_parameter<int>("csn", 0);
    declare_parameter<int>("ce", 0);
    declare_parameter<int>("irq", 0);
    declare_parameter<std::string>("pa", "low");
    declare_parameter<int>("channel", 0);

    radio_id_ = parse_radio_id(this->get_namespace());

    transmit_robots_ = get_parameter("robots").as_integer_array();
    int64_t transmit_hz = get_parameter("hz").as_int();

    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        "/referee/team_color", rclcpp::QoS(1).transient_local(),
        // NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const rj_msgs::msg::TeamColor::SharedPtr msg) {
            if (msg->is_blue != blue_team_) {
                blue_team_ = msg->is_blue;
            }
        });

    // Create the publishers and subscriptions for each robot
    for (const int64_t& robot_id : transmit_robots_) {
        motion_subs_[robot_id] = create_subscription<rj_msgs::msg::MotionSetpoint>(
            fmt::format("/control/motion_setpoint/robot_{}", robot_id), rclcpp::QoS(1),
            // NOLINTNEXTLINE(performance-unnecessary-value-param)
            [this, robot_id](const rj_msgs::msg::MotionSetpoint::SharedPtr msg) {
                motion_setpoints_[robot_id] = msg;
            });
        motion_setpoints_[robot_id] = std::make_shared<rj_msgs::msg::MotionSetpoint>();

        manipulator_subs_[robot_id] = create_subscription<rj_msgs::msg::ManipulatorSetpoint>(
            fmt::format("/control/manipulator_setpoint/robot_{}", robot_id), rclcpp::QoS(1),
            // NOLINTNEXTLINE(performance-unnecessary-value-param)
            [this, robot_id](const rj_msgs::msg::ManipulatorSetpoint::SharedPtr msg) {
                manipulator_setpoints_[robot_id] = msg;
            });
        manipulator_setpoints_[robot_id] = std::make_shared<rj_msgs::msg::ManipulatorSetpoint>();

        status_pubs_[robot_id] = create_publisher<rj_msgs::msg::RobotStatus>(
            fmt::format("/radio/robot_status/robot_{}", robot_id), rclcpp::QoS(1));
        robot_statuses_[robot_id] = std::nullopt;
    }

    // Bind the wall timer to send robot commands
    transmit_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / transmit_hz), [this] {
        publish_robot_statuses();
        send_motion_commands();
    });

    // Bind the callback handle
    parameter_callback_handle_ =
        add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params)
                                           -> rcl_interfaces::msg::SetParametersResult {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = parse_parameters(params);
            if (!result.successful) {
                result.reason = "Failed to parse parameters";
            }
            return result;
        });

    // Initialize the radio
    uint16_t csn = static_cast<uint16_t>(get_parameter("csn").as_int());
    uint16_t ce = static_cast<uint16_t>(
        get_parameter("ce").as_int());  // NOLINT(readability-identifier-length)
    uint8_t channel = static_cast<uint8_t>(get_parameter("channel").as_int());
    rf24_pa_dbm_e pa =
        decode_pa(get_parameter("pa").as_string());  // NOLINT(readability-identifier-length)

    RCLCPP_INFO(get_logger(),
                "Initializing RF24 radio with CE pin %d, CSN pin %d, channel %d, PA level %s", ce,
                csn, channel, get_parameter("pa").as_string().c_str());

    radio_ = RF24(ce, csn);
    if (!radio_.begin()) {
        throw std::runtime_error("Failed to initialize RF24 radio");
    }

    radio_.setPALevel(pa);
    radio_.maskIRQ(true, true, false);
    radio_.setRetries(0, 0);
    radio_.setAutoAck(false);
    radio_.setCRCLength(RF24_CRC_8);
    radio_.setChannel(channel);
    radio_.setDataRate(RF24_1MBPS);
    radio_.stopListening();
    radio_.openReadingPipe(1, rtp::kBaseStationAddresses[blue_team_ ? 0 : 1].data());

    chip_ = gpiod_chip_open("/dev/gpiochip0");
    if (!chip_) {
        throw std::runtime_error("Failed to open gpiochip0");
    }

    irq_ = static_cast<uint8_t>(get_parameter("irq").as_int());
    irq_line_ = gpiod_chip_get_line(chip_, irq_);
    if (!irq_line_) {
        throw std::runtime_error("Failed to get IRQ line");
    }

    struct gpiod_line_request_config config{};
    std::string consumer_name = fmt::format("radio_node_{}", radio_id_);
    config.consumer = consumer_name.c_str();
    config.request_type = GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE;
    config.flags = 0;

    if (gpiod_line_request(irq_line_, &config, 0) < 0) {
        throw std::runtime_error("Failed to request IRQ line events");
    }

    irq_thread_ = std::thread(&RFRadioNode::irq_loop, this);
}

RFRadioNode::~RFRadioNode() {
    running_ = false;
    if (irq_thread_.joinable()) {
        irq_thread_.join();
    }

    if (irq_line_) {
        gpiod_line_release(irq_line_);
    }

    if (chip_) {
        gpiod_chip_close(chip_);
    }
}

void RFRadioNode::send_motion_commands() {
    std::lock_guard<std::mutex> lock(rf24_mutex_);
    radio_.stopListening();
    radio_.setPayloadSize(rtp::ControlMessage::kSize);
    for (int64_t robot_id : transmit_robots_) {
        radio_.openWritingPipe(
            rtp::kRobotAddresses[blue_team_ ? 0 : 1][static_cast<uint64_t>(robot_id)].data());
        rtp::ControlMessage message = rtp::ControlMessage::from_ros(
            static_cast<unsigned int>(robot_id), blue_team_, motion_setpoints_[robot_id],
            manipulator_setpoints_[robot_id]);
        radio_.write(&message, rtp::ControlMessage::kSize);
    }
    radio_.openReadingPipe(1, rtp::kBaseStationAddresses[blue_team_ ? 0 : 1].data());
    radio_.startListening();
}

void RFRadioNode::publish_robot_statuses() {
    RCLCPP_INFO(get_logger(), "Publishing robot statuses for robots");
    for (int64_t robot_id : transmit_robots_) {
        auto pair = robot_statuses_.find(robot_id);
        if (pair != robot_statuses_.end() && pair->second.has_value()) {
            rj_msgs::msg::RobotStatus msg = pair->second->to_ros();
            status_pubs_[robot_id]->publish(msg);
            pair->second = std::nullopt;
        }
    }
}

void RFRadioNode::irq_loop() {
    struct gpiod_line_event event{};
    struct timespec timeout{0, 500'000'000};

    while (running_) {
        int ret = gpiod_line_event_wait(irq_line_, &timeout);
        if (ret < 0) {
            break;
        } else if (ret == 0) {
            continue;
        }

        if (gpiod_line_event_read(irq_line_, &event) == 0) {
            radio_gpio_callback();
        }
    }
}

void RFRadioNode::radio_gpio_callback() {
    std::lock_guard<std::mutex> lock(rf24_mutex_);
    radio_.clearStatusFlags(RF24_IRQ_ALL);
    uint8_t pipe = 0;
    while (radio_.available(&pipe)) {
        uint8_t bytes = radio_.getPayloadSize();
        if (bytes >= rtp::RobotStatusMessage::kSize) {
            rtp::RobotStatusMessage status = {};
            radio_.read(&status, rtp::RobotStatusMessage::kSize);
            robot_statuses_[status.robot_id] = status;
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

            std::unordered_set<int64_t> remove_ids;
            for (int64_t id : current_ids) {
                if (!new_ids.count(id)) remove_ids.insert(id);
            }

            for (int64_t robot_id : remove_ids) {
                motion_subs_.erase(robot_id);
                motion_setpoints_.erase(robot_id);
                manipulator_subs_.erase(robot_id);
                manipulator_setpoints_.erase(robot_id);
                status_pubs_.erase(robot_id);
                robot_statuses_.erase(robot_id);
            }

            std::unordered_set<int64_t> add_ids;
            for (int64_t id : new_ids) {
                if (!current_ids.count(id)) add_ids.insert(id);
            }

            for (int64_t robot_id : add_ids) {
                motion_subs_[robot_id] = create_subscription<rj_msgs::msg::MotionSetpoint>(
                    "robot_" + std::to_string(robot_id) + "/motion_setpoint", 10,
                    // NOLINTNEXTLINE(performance-unnecessary-value-param)
                    [this, robot_id](const rj_msgs::msg::MotionSetpoint::SharedPtr msg) {
                        motion_setpoints_[robot_id] = msg;
                    });
                motion_setpoints_[robot_id] = std::make_shared<rj_msgs::msg::MotionSetpoint>();

                manipulator_subs_[robot_id] =
                    create_subscription<rj_msgs::msg::ManipulatorSetpoint>(
                        "robot_" + std::to_string(robot_id) + "/manipulator_setpoint", 10,
                        // NOLINTNEXTLINE(performance-unnecessary-value-param)
                        [this, robot_id](const rj_msgs::msg::ManipulatorSetpoint::SharedPtr msg) {
                            manipulator_setpoints_[robot_id] = msg;
                        });
                manipulator_setpoints_[robot_id] =
                    std::make_shared<rj_msgs::msg::ManipulatorSetpoint>();

                status_pubs_[robot_id] = create_publisher<rj_msgs::msg::RobotStatus>(
                    "robot_" + std::to_string(robot_id) + "/status", 10);
                robot_statuses_[robot_id] = std::nullopt;
            }

        } else if (param.get_name() == "hz") {
            int64_t new_transmit_hz = get_parameter("hz").as_int();
            transmit_timer_->cancel();
            transmit_timer_ =
                create_wall_timer(std::chrono::milliseconds(1000 / new_transmit_hz), [this] {
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
        } else if (param.get_name() == "id" || param.get_name() == "ce" ||
                   param.get_name() == "csn" || param.get_name() == "irq") {
            return false;
        }
        return true;
    });
}

// NOLINTNEXTLINE(readability-identifier-length)
uint8_t RFRadioNode::parse_radio_id(const std::string& ns) {
    // Strip any leading slash
    std::string name = (!ns.empty() && ns[0] == '/') ? ns.substr(1) : ns;

    const std::string prefix = "radio_";
    if (name.rfind(prefix, 0) != 0) {
        return 0;
    }

    std::string id_str = name.substr(prefix.size());
    if (id_str.empty()) {
        return 0;
    }

    // Ensure all remaining characters are digits
    // NOLINTNEXTLINE(readability-identifier-length)
    for (char c : id_str) {
        if (std::isdigit(static_cast<unsigned char>(c)) == 0) {
            return 0;
        }
    }

    return std::stoi(id_str);
}

}  // namespace base_station

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<base_station::RFRadioNode> node;
    try {
        node = std::make_shared<base_station::RFRadioNode>();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
