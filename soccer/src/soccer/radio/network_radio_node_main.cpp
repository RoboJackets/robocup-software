#include <rj_common/network.hpp>

#include "network_radio.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto radio = std::make_shared<radio::NetworkRadio>(kNetworkRadioServerPort);
    rclcpp::spin(radio);
}
