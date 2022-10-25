#pragma once

#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/team_color.hpp>
#include <rj_msgs/msg/team_fruit.hpp>

//#include "robot_intent.hpp"
//#include "robot_status.hpp"

//namespace soccer_mom {

/*
constexpr auto kRadioParamModule = "radio";
DECLARE_FLOAT64(kRadioParamModule, timeout);
*/

/**
 * @brief Sends and receives information to/from our robots.
 *
 * @details This is the abstract superclass for NetworkRadio and SimRadio, which do
 * the actual work - this just declares the interface and handles sending stop commands when no new
 * commands come in for a while.
 */
class SoccerMom : public rclcpp::Node {
public:
    SoccerMom();

//protected:
//    void publish(const rj_msgs::msg::TeamFruit& color);

//    virtual void send(int robot_id, const rj_msgs::msg::MotionSetpoint& motion,
//                      const rj_msgs::msg::ManipulatorSetpoint& manipulator) = 0;
//    virtual void receive() = 0;
//    virtual void switch_team(bool blue) = 0;

    // should be used in both subclasses
    // (Kevin can't find where network radio gets its IP from so currently is only sim radio)
//    std::string param_radio_interface_;

private:
      void publish(const rj_msgs::msg::TeamColor::SharedPtr color);
//    void tick();

//    std::array<rclcpp::Publisher<rj_msgs::msg::RobotStatus>::SharedPtr, kNumShells>
//        robot_status_pubs_;
//    std::array<rclcpp::Subscription<rj_msgs::msg::MotionSetpoint>::SharedPtr, kNumShells>
//        motion_subs_;
//    std::array<rclcpp::Subscription<rj_msgs::msg::ManipulatorSetpoint>::SharedPtr, kNumShells>
//        manipulator_subs_;
    rclcpp::Publisher<rj_msgs::msg::TeamFruit>::SharedPtr team_fruit_pub_;
    rclcpp::Subscription<rj_msgs::msg::TeamColor>::SharedPtr team_color_sub_;
//    rclcpp::TimerBase::SharedPtr tick_timer_;

//    std::array<rj_msgs::msg::ManipulatorSetpoint, kNumShells> manipulators_cached_;
//    std::array<RJ::Time, kNumShells> last_updates_ = {};

//    ::params::LocalROS2ParamProvider param_provider_;
};

//}  // namespace soccer_mom
