#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <chrono>

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <boost/asio.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/network.hpp>
#include <rj_common/radio/packet_convert.hpp>
#include <rj_common/time.hpp>
#include <rj_msgs/msg/alive_robots.hpp>
#include <rj_msgs/srv/sim_placement.hpp>
#include <rj_param_utils/global_params.hpp>
#include <rj_protos/ssl_simulation_control.pb.h>
#include <rj_protos/ssl_simulation_robot_control.pb.h>
#include <rj_protos/ssl_simulation_robot_feedback.pb.h>
#include <rj_utils/logging.hpp>

#include "rj_radio/radio.hpp"

using namespace std::chrono_literals;

class SoccerMom : public rclcpp::Node
{
public:
    SoccerMom() : Node("soccer_mom")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/soccer_mom", 10);

        team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            last_is_blue_ = color->is_blue;
        });
        
        timer_ = this->create_wall_timer(
            500ms, std::bind(&SoccerMom::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Soccer Mom has been started.");
    }

private:
    void timer_callback()
    {
        std_msgs::msg::String message;

        message.data = (*last_is_blue_) ? "blueberries" : "bananas";

        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::optional<bool> last_is_blue_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SoccerMom>());
    rclcpp::shutdown();
    return 0;
}