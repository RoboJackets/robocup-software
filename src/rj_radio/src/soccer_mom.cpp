#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <std_msgs/msg/string.hpp>

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom() : Node{"soccer_mom"} {
        fruit_pub_ = create_publisher<std_msgs::msg::String>("/team_fruit", rclcpp::QoS(1));
        team_color_sub_ = create_subscription<rj_msgs::msg::TeamColor>(
            referee::topics::kTeamColorTopic, rclcpp::QoS(1).transient_local(),
            [this](rj_msgs::msg::TeamColor::SharedPtr team_color) {
                std_msgs::msg::String fruit;
                fruit.data = team_color->is_blue ? "blueberries" : "banana";
                fruit_pub_->publish(fruit);
            });
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fruit_pub_;
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
};

}  

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tutorial::SoccerMom>());
    rclcpp::shutdown();
    return 0;
}
