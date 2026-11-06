#include "rj_radio/soccer_mom.hpp"

namespace tutorial {
    SoccerMom::SoccerMom() : Node("soccer_mom") {
        
        const auto keep_latest =  rclcpp::QoS(1).transient_local();  

        fruit_pub_ = create_publisher<std_msgs::msg::String>("/team_fruit", keep_latest);

        team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, keep_latest,
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {  // NOLINT
            publish_fruit(color);
        });
    }

    void SoccerMom::publish_fruit(rj_msgs::msg::TeamColor::SharedPtr color) {
        std_msgs::msg::String fruit;
        if (color->is_blue) {
            fruit.data = "blueberries";
        } else {
            fruit.data = "banana";
        }

        fruit_pub_->publish(fruit);

    }


}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto soccer_mom = std::make_shared<tutorial::SoccerMom>();
    rclcpp::spin(soccer_mom);
}