#include "soccermom.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tutorial::SoccerMom>());
  rclcpp::shutdown();
  return 0;
}
