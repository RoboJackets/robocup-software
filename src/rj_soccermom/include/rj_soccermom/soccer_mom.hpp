#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_constants/topic_names.hpp>


class SoccerMomNode : public rclcpp::Node 
{
    public:
        SoccerMomNode();

    private:
        void team_color_callback(rj_msgs::msg::TeamColor::SharedPtr msg);

        rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr soccer_mom_pub_;
};
