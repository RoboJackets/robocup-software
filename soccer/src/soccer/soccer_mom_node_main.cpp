#include "soccer_mom.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tutorial::Soccer_Mom>());
}