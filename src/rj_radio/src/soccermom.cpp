#include "rj_radio/soccermom.hpp"

namespace rj_radio {

SoccerMomNode::SoccerMomNode()
    : Node("soccermom")
{
    // Publisher for /team_fruit
    fruit_pub_ = this->create_publisher<std_msgs::msg::String>("/team_fruit", 10);

    // Subscriber to team color
    team_color_sub_ = this->create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic,
        10,
        std::bind(&SoccerMomNode::team_color_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "SoccerMom node started.");
}

void SoccerMomNode::team_color_callback(const rj_msgs::msg::TeamColor::SharedPtr msg)
{
    std_msgs::msg::String fruit;
    if (msg->is_blue) {
        fruit.data = "blueberries";
    } else {
        fruit.data = "banana";
    }

    RCLCPP_INFO(this->get_logger(), "Publishing fruit: %s", fruit.data.c_str());
    fruit_pub_->publish(fruit);
}

}  // namespace rj_radio

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rj_radio::SoccerMomNode>());
    rclcpp::shutdown();
    return 0;
}
