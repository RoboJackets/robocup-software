#include <rj_param_utils/global_param.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_vision_receiver/vision_receiver.hpp>

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "global_params", 50, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      // TODO: add print to rj log
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char**argv) {
    rclcpp::init(argc, argv);
    
    // create global node
    rclcpp::Node* global_param_node = std::make_shared<rclcpp::Node>(params::kGlobalParamNodeName);
    global_param_node->declare_parameter("global_bool", false);
    global_param_node->declare_parameter("global_float", 3.14f);
    global_param_node->declare_parameter("global_string", "I am a string.");
    rclcpp::spin(global_param_node); 

    // Create vision receiver node
    rclcpp::Node* vision_receiver_node = std::make_shared<vision_receiver::VisionReceiver()>();
    
    rclcpp::Node* subscriber = std::make_shared<MinimalSubscriber::MinimalSubscriber()>(); 

    // TODO: delay 1s
    
    global_param_node->set_parameter("global_bool", true);
    global_param_node->set_parameter("global_float", 6.28f);
    global_param_node->set_parameter("global_string", "I am a big STRING.");
    
    rclcpp::shutdown();
    return 0;
}
