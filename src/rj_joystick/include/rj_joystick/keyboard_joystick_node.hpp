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


    static const std::map<SDL_Scancode, AxisBinding> kAxisMap = {
        {SDL_SCANCODE_W, {axis::kLeftX, -1.0f}},
        {SDL_SCANCODE_S, {axis::kLeftX, 1.0f}},
        {SDL_SCANCODE_A, {axis::kLeftY, -1.0f}},
        {SDL_SCANCODE_D, {axis::kLeftY, 1.0f}},
        {SDL_SCANCODE_Q, {axis::kRightX, -1.0f}},
        {SDL_SCANCODE_E, {axis::kRightX, 1.0f}},
        {SDL_SCANCODE_K, {axis::kTriggerRight, 1.0f}},
    };

    static const std::map<SDL_Scancode, size_t> kButtonMap = {
        {SDL_SCANCODE_J, button::kLeftShoulder},
        {SDL_SCANCODE_LSHIFT, button::kLeftStick},
        {SDL_SCANCODE_SPACE, button::kRightShoulder},
        {SDL_SCANCODE_RSHIFT, button::kRightShoulder},
        {SDL_SCANCODE_UP, button::kDpadUp},
        {SDL_SCANCODE_DOWN, button::kDpadDown},
        {SDL_SCANCODE_LEFT, button::kDpadLeft},
        {SDL_SCANCODE_RIGHT, button::kDpadRight},
    };

    SDL_Window* window_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace joystick