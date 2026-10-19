#include "rj_radio/soccer_mom.hpp"

using namespace std;
using namespace std::chrono_literals;

namespace tutorial {

// constructor for the SoccerMom class
SoccerMom::SoccerMom() : Node("soccer_mom") {
    // intialize the publisher to publish to team_fruit
    publisher_ = this->create_publisher<std_msgs::msg::String>("team_fruit", 10);
    // initialize the subscription to subscribe to team_color published by the referee node
    subscription_ = this->create_subscription<rj_msgs::msg::TeamColor>(
        "/referee/team_color", 10,
        std::bind(&SoccerMom::topic_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(500ms, std::bind(&SoccerMom::timer_callback, this));
}

// published the team color to the topic team_fruit
void SoccerMom::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = team_color_;
    publisher_->publish(message);
}

// subscribes to the team_color topic and updates the variable
void SoccerMom::topic_callback(const rj_msgs::msg::TeamColor::SharedPtr msg) {
    team_color_ = msg->is_blue ? "blueberries" : "bananas";
}
}  // namespace tutorial

// main function, runs the node
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tutorial::SoccerMom>());
    rclcpp::shutdown();
    return 0;
}
