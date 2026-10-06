#include "rj_radio/soccer_mom.hpp"

namespace tutorial {

SoccerMom::SoccerMom() : Node("SoccerMom") {
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>("/referee/team_color", 
        10, 
        [this](rj_msgs::msg::TeamColor::SharedPtr color) {
            publish_team_fruit(color->is_blue);
        }
    );

    publisher_ = this->create_publisher<std_msgs::msg::String>("/team_fruit", 10);

}


void SoccerMom::publish_team_fruit(bool is_blue) {
    std_msgs::msg::String fruit;
    if(is_blue) {
        fruit.data = "blueberries";
    } else {
        fruit.data = "banana";
    }
    publisher_->publish(fruit); 
}


}  // namespace tutorial

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto soccer_mom = std::make_shared<tutorial::SoccerMom>();
    rclcpp::spin(soccer_mom);
    rclcpp::shutdown();
}