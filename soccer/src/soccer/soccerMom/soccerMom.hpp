#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rj_constants/topic_names.hpp>
#include <string>
#include <config_client/config_client.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_utils/concurrent_queue.hpp>

namespace tutorial {
    class SoccerMom : public rclcpp::Node {
        public:
            SoccerMom();
        private:
            std_msgs::msg::String teamFruit;
            void teamColorCallBack(bool isBlue);
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr team_fruit_publisher;
            rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
    };
}