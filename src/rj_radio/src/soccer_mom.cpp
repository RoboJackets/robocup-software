
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


class SoccerMom: public rclcpp::Node
{
    public:
    SoccerMom() : Node("soccer_mom")
    {
        Publisher_ = this->create_publisher<std_msgs::msg::String>("/soccer_mom",10);
        Subscriber_ = create_subscription<rj_msgs::msg::TeamColor>(
            referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
            [this](rj_msgs::msg::TeamColor::SharedPtr color) {
                is_blue = color->is_blue;
            }
        );
        auto timer_callback =
            [this]() -> void {
                auto message = std_msgs::msg::String();
                message.data = (is_blue) ? "blueberries" : "banananas";
                this->Publisher_->publish(message);
      };

        timer_ = this->create_wall_timer(500ms, timer_callback);

    }

    private:
        rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr Subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Publisher_;
        bool is_blue;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SoccerMom>());
    rclcpp::shutdown();
    return 0;
}



