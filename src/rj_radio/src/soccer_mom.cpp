#include "rj_radio/soccer_mom.hpp"

namespace Tutorial {
    SoccerMom::SoccerMom() : Node("soccer_mom") {
        fruitPublisher_ = create_publisher<std_msgs::msg::String>("/team_fruit", 10);

        std::function<void(rj_msgs::msg::TeamColor::SharedPtr)> subCallback = [](rj_msgs::msg::TeamColor::SharedPtr msg) {
            isBlue = msg->is_blue;
        };
        colorSubscriber_ = create_subscription<rj_msgs::msg::TeamColor>("/referee/team_color", 10, subCallback);\
        
        std::function<void()> timerCallback = []() {
            std_msgs::msg::String msg;
            if (isBlue) {
                msg.data = "blueberries";
            } else {
                msg.data = "banana";
            }
            fruitPublisher_->publish(msg);
        };
        timer_ = create_wall_timer(1000ms, timerCallback);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tutorial::SoccerMom>());
    rclcpp::shutdown();
    return 0;
}