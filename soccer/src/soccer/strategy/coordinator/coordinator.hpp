#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

namespace strategy {

/**
 * @brief Base class for coordinators using CRTP pattern
 * 
 * @tparam Derived The derived coordinator class (e.g. KickerPicker)
 * @tparam ServiceT The ROS service type (e.g. rj_msgs::srv::KickerPicker)
 * @tparam TopicT The ROS topic type (e.g. rj_msgs::msg::KickerPicker)
 * 
 * To implement a new coordinator:
 * 1. Inherit from this class, passing your class as the Derived template parameter
 * 2. Implement the service_callback method with this signature:
 *    void service_callback(RequestPtr request, ResponsePtr response)
 * 
 * Example:
 * @code
 * class MyCoordinator : public Coordinator<MyCoordinator, MyService, MyTopic> {
 * public:
 *     MyCoordinator() : Coordinator("my_service", "my_topic") {}
 *     
 *     void service_callback(RequestPtr request, ResponsePtr response) {
 *         // Handle the service request
 *         response->success = true;
 *     }
 * };
 * @endcode
 */
template <class Derived, class ServiceT, class TopicT>
class Coordinator : public rclcpp::Node {
public:
    // Type aliases to simplify derived class implementations
    using RequestPtr = std::shared_ptr<typename ServiceT::Request>;
    using ResponsePtr = std::shared_ptr<typename ServiceT::Response>;

    Coordinator(const std::string& service_name, const std::string& topic_name)
        : Node(std::string(typeid(Derived).name())) {
        service_ = this->create_service<ServiceT>(
            service_name, [this](RequestPtr request, ResponsePtr response) {
                static_cast<Derived*>(this)->service_callback(request, response);
            });
            
        const auto qos = rclcpp::QoS(1).best_effort().transient_local();
        publisher_ = this->create_publisher<TopicT>(topic_name, qos);
    }
    
    virtual ~Coordinator() = default;

protected:
    typename rclcpp::Service<ServiceT>::SharedPtr service_;
    typename rclcpp::Publisher<TopicT>::SharedPtr publisher_;
};

}  // namespace strategy
