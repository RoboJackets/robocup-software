#include "rj_radio/soccer_mom.hpp"

namespace tutorial {

SoccerMom::SoccerMom()
    : Node{"soccer_mom", rclcpp::NodeOptions{}
                        .automatically_declare_parameters_from_overrides(true)
                        .allow_undeclared_parameters(true)}
{

        team_fruit_pub_ =
        create_publisher<std_msgs::msg::String>(topics::kTeamFruitTopic, rclcpp::QoS(1));

        team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
            referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
            [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
                std_msgs::msg::String fruit;

                if (color->is_blue) {
                    fruit.data = "blueberries";
                } else {
                    fruit.data = "bananas";
                }

                team_fruit_pub_->publish(fruit);
            });
}

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tutorial::SoccerMom>());
    rclcpp::shutdown();
    return 0;
}