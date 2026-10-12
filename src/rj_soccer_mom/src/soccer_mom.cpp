#include "soccer_mom.hpp"

namespace tutorial {
    SoccerMom::SoccerMom() : Node("soccer_mom") {
        // publisher to /team_fruit
        fruit_pub_ = this->create_publisher<std_msgs::msg::String>("/team_fruit", 10);

        // subscriber - listem to team color
        // creating subscriber & LAMBDA FUNCTION
        team_color_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/team_color",
            10,
            // THIS IS LAMBDA SYNTAX
            [this](const std_msgs::msg::String::SharedPtr msg) {
                auto fruit_msg = std_msgs::msg::String();

                // conditions if yello banana , if blu, boobetty
                if (msg->data == "yellow") {
                    fruit_msg.data = "banana";
                } else if (msg->data == "blue") {
                    fruit_msg.data = "blueberry";
                }

                if (!fruit_msg.data.empty()) {
                    fruit_pub_->publish(fruit_msg);
                    RCLCPP_INFO(this->get_logger(), "Published fruit: %s", fruit_msg.data.c_str());
                }
            }
        );
    }
} // namespace tut


// MAIN
int main (int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tutorial::SoccerMom>());
    rclcpp::shutdown();
    return 0;
}