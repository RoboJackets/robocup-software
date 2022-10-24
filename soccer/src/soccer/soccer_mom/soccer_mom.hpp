#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <rj_msgs/msg/team_color.hpp>
#include <rj_constants/topic_names.hpp>

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();
private:
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr team_fruit_pub_;
    void pick_fruit(bool is_blue);
};

}  // namespace tutorial