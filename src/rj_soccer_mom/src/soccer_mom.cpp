#include <rj_constants/topic_names.hpp>
#include <rj_soccer_mom/soccer_mom.hpp>

using namespace std::chrono_literals;

SoccerMom::SoccerMom() : Node("soccer_mom") {
    subscription_ = this->create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            if (color->is_blue != is_blue) {
                is_blue = color->is_blue;
            }
        });

    publisher_ = this->create_publisher<std_msgs::msg::String>("team_fruit", 10);

    auto timer_callback = [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = is_blue ? "blueberries" : "banana";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->publisher_->publish(message);
    };
    timer_ = this->create_wall_timer(1000ms, timer_callback);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SoccerMom>());
    rclcpp::shutdown();
    return 0;
}