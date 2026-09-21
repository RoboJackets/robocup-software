#include "rj_radio/soccer_mom.hpp"
#include <chrono>

using namespace std::chrono_literals;



namespace tutorial {
    Soccer_Mom::Soccer_Mom() : Node{"soccer_mom", rclcpp::NodeOptions{}
                        .automatically_declare_parameters_from_overrides(true)
                        .allow_undeclared_parameters(true)}{
        publisher_ = create_publisher<std_msgs::msg::String>("team_fruit",rclcpp::QoS(1));
        subscriber_ = create_subscription<rj_msgs::msg::TeamColor>("referee/team_color", 10,
             std::bind(&Soccer_Mom::subscriber_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(500ms,std::bind(&Soccer_Mom::publisher_callback,this));
        
    }

    void Soccer_Mom::publisher_callback(){
        auto message = std_msgs::msg::String();
        if (is_blue){
            message.data = "blueberries";
        } else {
            message.data = "banana";
        }
        //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    void Soccer_Mom::subscriber_callback(rj_msgs::msg::TeamColor::SharedPtr message) {
        Soccer_Mom::is_blue = message->is_blue;
    }


}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<tutorial::Soccer_Mom>());
    return 0;
}