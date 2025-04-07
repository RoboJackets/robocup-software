#pragma once

#include <memory>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

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
    using ServiceType = ServiceT;
    using TopicType = TopicT;

    template <typename... Args>
    Coordinator(const std::string& service_name, const std::string& topic_name, Args&&... node_args)
        : rclcpp::Node(std::forward<Args>(node_args)...),
          service_(this->create_service<ServiceT>(service_name,
                                                  [this](RequestPtr request, ResponsePtr response) {
                                                      static_cast<Derived*>(this)->service_callback(
                                                          request, response);
                                                  })),
          publisher_(this->create_publisher<TopicT>(
              topic_name, rclcpp::QoS(1).best_effort().transient_local())) {}

    ~Coordinator() override = default;

    // In general, nodes are neither copyable nor moveable.
    Coordinator(const Coordinator&) = delete;
    Coordinator& operator=(const Coordinator&) = delete;
    Coordinator(Coordinator&&) = delete;
    Coordinator& operator=(Coordinator&&) = delete;

protected:
    // Type aliases to simplify derived class implementations
    using RequestPtr = std::shared_ptr<typename ServiceT::Request>;
    using ResponsePtr = std::shared_ptr<typename ServiceT::Response>;

    typename rclcpp::Service<ServiceT>::SharedPtr service_;
    typename rclcpp::Publisher<TopicT>::SharedPtr publisher_;
};

}  // namespace strategy
