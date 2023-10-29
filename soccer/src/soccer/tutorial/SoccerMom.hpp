#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_msgs/srv/quick_commands.hpp>
#include <rj_msgs/srv/quick_restart.hpp>

#include "rclcpp/rclcpp.hpp"
#include "referee/referee_base.hpp"
#include "std_msgs/msg/string.hpp"

namespace tutorial {
class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

private:
    std_msgs::msg::String tf;
    void teamColorCallback(bool isBlue);
    std::string topic;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _team_fruit_pub;
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr _team_color_sub;
};
}  // namespace tutorial