
#include "global_params.hpp"
#include "soccer_mom.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto soccer_mom = std::make_shared<tutorial::SoccerMom>();

    rclcpp::spin(soccer_mom);
}