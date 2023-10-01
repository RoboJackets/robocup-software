#include "soccermom.hpp"

#include <spdlog/spdlog.h>

namespace tutorial {

SoccerMom :: SoccerMom() : Node("soccer_mom") {
       publisher_ = this->create_publisher<std_msgs::msg::String>("team_fruit", 10);
       subscription_ = this->create_subscription<rj_msgs::msg::TeamColor>(
      "referee/team_color", 10, [this](rj_msgs::msg::TeamColor::SharedPtr color) {
          is_blue_ = color->is_blue;
          });
       timer_ = create_wall_timer(std::chrono::milliseconds(16), [this]() { send(); });
    }


void SoccerMom :: send() {
  {
    auto message = std_msgs::msg::String();
    if (is_blue_) {
        message.data = "Blueberry";
    } else {
        message.data = "Banana";
    }
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
}

}
