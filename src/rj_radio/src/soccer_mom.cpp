#include "rj_radio/soccer_mom.hpp"

#include <rj_constants/topic_names.hpp>

// The header lists what SoccerMom is. This file is the code that makes it work.

namespace tutorial {

SoccerMom::SoccerMom() : Node("soccer_mom") {
    // QoS is a setting for how messages are sent.
    // QoS(1) means "only keep the newest message."
    // transient_local means a subscriber that starts later still gets that newest message.
    // The referee publishes team color with these same settings, so this subscription matches it.
    const auto keep_latest = rclcpp::QoS(1).transient_local();

    // Make the publisher before the subscription. Then we can send a fruit as soon as a color
    // arrives. "/team_fruit" is a new topic that only this node publishes.
    fruit_pub_ = create_publisher<std_msgs::msg::String>("/team_fruit", keep_latest);

    // kTeamColorTopic is the shared name "referee/team_color".
    // The part in [this](...) { ... } is a lambda: a small function with no name.
    // ROS runs that function for every team-color message, and we pass the message along.
    team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
        referee::topics::kTeamColorTopic, keep_latest,
        [this](rj_msgs::msg::TeamColor::SharedPtr color) { on_team_color(color); });
}

void SoccerMom::on_team_color(const rj_msgs::msg::TeamColor::SharedPtr& color) {
    // TeamColor has one field, is_blue.
    // true means we are the blue team. false means we are the yellow team.
    // The fruit is stored in the "data" field of a normal ROS string message.
    std_msgs::msg::String fruit;
    if (color->is_blue) {
        fruit.data = "blueberries";
    } else {
        fruit.data = "banana";
    }

    // The arrow (->) is how you call a method when you have a pointer to an object.
    // fruit_pub_ points at our publisher.
    fruit_pub_->publish(fruit);
}

}  // namespace tutorial

// This is the start of the program. The launch file runs this executable.
int main(int argc, char** argv) {
    // Connect this program to ROS. Do this before creating the node.
    rclcpp::init(argc, argv);

    // Create the node, then spin it.
    // Spin waits for messages and calls on_team_color when one arrives.
    // It returns when the rest of the stack shuts down.
    auto soccer_mom = std::make_shared<tutorial::SoccerMom>();
    rclcpp::spin(soccer_mom);

    rclcpp::shutdown();
    return 0;
}
