#include "src/rj_radio/include/soccermom.hpp"

namespace tutorial {

SoccerMom::SoccerMom() : Node("soccer_mom") {
    // Create the publisher on the /team_fruit topic
    fruit_pub_ = this->create_publisher<std_msgs::msg::String>("/team_fruit", 10);

    // Create the subscriber on the /referee/team_color topic
    team_color_sub_ = this->create_subscription<rj_msgs::msg::TeamColor>(
        "/referee/team_color", 10,
        [this](const rj_msgs::msg::TeamColor::SharedPtr msg) {
            color_callback(msg);
        });
}

void SoccerMom::color_callback(const rj_msgs::msg::TeamColor::SharedPtr msg) {
    std_msgs::msg::String fruit_msg;
    
    // Check the boolean we found using the ROS 2 interface command
    if (msg->is_blue) {
        fruit_msg.data = "blueberries";
    } else {
        fruit_msg.data = "banana";
    }
    
    // Publish the fruit!
    fruit_pub_->publish(fruit_msg);
}

}  // namespace tutorial

// Standard ROS 2 main execution block
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tutorial::SoccerMom>());
    rclcpp::shutdown();
    return 0;
}