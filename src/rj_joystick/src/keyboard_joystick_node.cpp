#include "rj_joystick/keyboard_joystick_node.hpp"

#include <rj_common/ros_params.hpp>

namespace joystick {

KeyboardJoystickNode::KeyboardJoystickNode() : rclcpp::Node("keyboard_joy") {
    const auto interval =
        declare_required_param<double>(*this, "publish_interval", "Joy publish interval, s");

    left_joy_percent_ = declare_required_param<double>(*this, "left_joy_percent", "Left joystick percent");
    left_joy_damped_percent_ = declare_required_param<double>(*this, "left_joy_damped_percent", "Left joystick damped percent");
    right_joy_percent_ = declare_required_param<double>(*this, "right_joy_percent", "Right joystick percent");
    right_joy_damped_percent_ = declare_required_param<double>(*this, "right_joy_damped_percent", "Right joystick damped percent");

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
        throw std::runtime_error(SDL_GetError());
    }
    window_ = SDL_CreateWindow("keyboard_joy", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 300,
                               300, 0);

    pub_ = create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(interval), [this] { tick(); });
}

KeyboardJoystickNode::~KeyboardJoystickNode() {
    SDL_DestroyWindow(window_);
    SDL_Quit();
}

void KeyboardJoystickNode::tick() {
    SDL_Event event;
    while (SDL_PollEvent(&event) != 0) {
        if (event.type == SDL_QUIT) {
            rclcpp::shutdown();
            return;
        }
    }

    const uint8_t* keys = SDL_GetKeyboardState(nullptr);

    sensor_msgs::msg::Joy msg;
    msg.header.stamp = now();
    msg.axes.assign(kNumAxes, 0.0f);
    msg.buttons.assign(kNumButtons, 0);

    for (const auto& [key, binding] : kAxisMap) {
        if (keys[key] != 0) {
            msg.axes[binding.index] += binding.value;
        }
    }
    for (const auto& [key, index] : kButtonMap) {
        if (keys[key] != 0) {
            msg.buttons[index] = 1;
        }
    }

    const bool damped = keys[SDL_SCANCODE_LSHIFT] != 0;
    const float left = damped ? left_joy_damped_percent_ : left_joy_percent_;
    const float right = damped ? right_joy_damped_percent_ : right_joy_percent_;

    msg.axes[axis::kLeftX] *= left;
    msg.axes[axis::kLeftY] *= left;
    msg.axes[axis::kRightX] *= right;

    pub_->publish(msg);
}

}  // namespace joystick

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<joystick::KeyboardJoystickNode>());
    rclcpp::shutdown();
}