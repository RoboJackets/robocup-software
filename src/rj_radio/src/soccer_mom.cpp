#include "rj_radio/soccer_mom.hpp"

namespace radio {
    DECLARE_FLOAT64(kRadioParamModule, timeout);
    SoccerMom::SoccerMom()
    : Node("soccer_mom", rclcpp::NodeOptions{}
                          .automatically_declare_parameters_from_overrides(true)
                          .allow_undeclared_parameters(true)),
      param_provider_(this, kRadioParamModule) {
        fruit_pub_ = create_publisher<rj_msgs::msg::FruitType>("/team_fruit", 10);
        team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            if (color->is_blue) {
                auto message = rj_msgs::msg::FruitType();
                message.fruit_type = "blueberries";
                fruit_pub_->publish(message);
            } else {
                auto message = rj_msgs::msg::FruitType();
                message.fruit_type = "banana";
                fruit_pub_->publish(message);
            }
        });
    }
}