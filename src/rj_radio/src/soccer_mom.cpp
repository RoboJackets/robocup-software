#include "rj_radio/soccer_mom.hpp"

using namespace std;

namespace tutorial {

class SoccerMom : public rclcpp::Node {
public:
    SoccerMom*() : Node("soccer_mom") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("team_fruit", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&SoccerMom::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Test";
        publisher_->publish(message);
    }
};
}  // namespace tutorial