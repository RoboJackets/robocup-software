#include <rj_common/network.hpp>

#include "sim_radio.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto radio = std::make_shared<radio::SimRadio>(kNetworkRadioServerPort);
    rclcpp::spin(radio);
}
