#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace strategy {

/**
 * @brief Abstract base class for coordinators
 *
 * @details Coordinators
 */
template <class ServiceT, class TopicT>
class Coordinator : public rclcpp::Node {
public:
    Coordinator(const std::string& service_name, const std::string& topic_name) {
        service_ = rclcpp::create_service<ServiceT>(
            service_name, [this](const std::shared_ptr<ServiceT::Request> request,
                                 const std::shared_ptr<ServiceT::Response> response) {
                service_callback(request, response);
            });
    }
    virtual ~Coordinator() = default;

protected:
    rclcpp::Service<ServiceT>::SharedPtr service_;
    rclcpp::Publisher<TopicT>::SharedPtr publisher_;

    virtual service_callback(ServiceT::Request request, ServiceT::Response response) = 0;
};

}  // namespace strategy
