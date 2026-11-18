#pragma once

#include <map>

#include <SDL2/SDL.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "rj_joystick/input_map.hpp"

namespace joystick {

class KeyboardJoystickNode : public rclcpp::Node {
public:
    KeyboardJoystickNode();
    ~KeyboardJoystickNode() override;

private:
    void tick();

    static constexpr size_t kNumAxes = 6;
    static constexpr size_t kNumButtons = 21;

    struct AxisBinding {
        size_t index;
        float value;
    };

    double left_joy_percent_;
    double left_joy_damped_percent_;
    double right_joy_percent_;
    double right_joy_damped_percent_;


    static const std::map<SDL_Scancode, AxisBinding> kAxisMap;
    static const std::map<SDL_Scancode, size_t> kButtonMap;

    SDL_Window* window_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace joystick