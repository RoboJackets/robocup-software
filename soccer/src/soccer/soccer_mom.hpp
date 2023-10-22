#include <rclcpp/rclcpp.hpp>
#include <rj_constants/topic_names.hpp>
#include <std_msgs/msg/string.hpp>
#include <rj_msgs/msg/team_color.hpp>


namespace tutorial {
class Soccer_Mom : public rclcpp::Node {
public:
	Soccer_Mom();

private:
	void send();
	void recieve(bool b);
	rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	bool is_blue_;
};
}

