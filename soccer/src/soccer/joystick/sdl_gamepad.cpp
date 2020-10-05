#include <algorithm>
#include <vector>

#include <spdlog/spdlog.h>

#include <joystick/sdl_gamepad.hpp>
#include <rj_utils/logging.hpp>

namespace joystick {
SDLGamepad::SDLGamepad(int device_index) {
    controller_ = SDL_GameControllerOpen(device_index);

    if (controller_ == nullptr) {
        FATAL_THROW("ERROR: Could not open controller! SDL Error: {}", SDL_GetError());
    }

    auto* joystick = SDL_GameControllerGetJoystick(controller_);

    name = std::string{SDL_JoystickName(joystick)};
    instance_id = SDL_JoystickInstanceID(joystick);
    guid = SDL_JoystickGetGUID(joystick);
    unique_id = SDLGamepad::get_unique_id(guid);

    state_.unique_id = unique_id;
}

SDLGamepad::~SDLGamepad() {
    if (controller_ != nullptr) {
        SDL_GameControllerClose(controller_);
        controller_ = nullptr;
    }
}

bool SDLGamepad::get_button(SDL_GameControllerButton button) {
    return static_cast<bool>(SDL_GameControllerGetButton(controller_, button));
}

int32_t SDLGamepad::get_axis(SDL_GameControllerAxis axis) {
    return SDL_GameControllerGetAxis(controller_, axis);
}

const GamepadMessage& SDLGamepad::update() {
    state_.update_time = RJ::now();

    // Sticks
    state_.sticks.left.x = get_axis(SDL_CONTROLLER_AXIS_LEFTX);
    state_.sticks.left.y = get_axis(SDL_CONTROLLER_AXIS_LEFTY);
    state_.sticks.right.x = get_axis(SDL_CONTROLLER_AXIS_RIGHTX);
    state_.sticks.right.y = get_axis(SDL_CONTROLLER_AXIS_RIGHTY);

    // Triggers
    state_.triggers.left = get_axis(SDL_CONTROLLER_AXIS_TRIGGERLEFT);
    state_.triggers.right = get_axis(SDL_CONTROLLER_AXIS_TRIGGERRIGHT);

    // DPAD
    state_.dpad.up = get_button(SDL_CONTROLLER_BUTTON_DPAD_UP);
    state_.dpad.down = get_button(SDL_CONTROLLER_BUTTON_DPAD_DOWN);
    state_.dpad.left = get_button(SDL_CONTROLLER_BUTTON_DPAD_LEFT);
    state_.dpad.right = get_button(SDL_CONTROLLER_BUTTON_DPAD_RIGHT);

    // Buttons
    state_.buttons.a = get_button(SDL_CONTROLLER_BUTTON_A);
    state_.buttons.b = get_button(SDL_CONTROLLER_BUTTON_B);
    state_.buttons.x = get_button(SDL_CONTROLLER_BUTTON_X);
    state_.buttons.y = get_button(SDL_CONTROLLER_BUTTON_Y);
    state_.buttons.back = get_button(SDL_CONTROLLER_BUTTON_BACK);
    state_.buttons.guide = get_button(SDL_CONTROLLER_BUTTON_GUIDE);
    state_.buttons.start = get_button(SDL_CONTROLLER_BUTTON_START);
    state_.buttons.left_stick = get_button(SDL_CONTROLLER_BUTTON_LEFTSTICK);
    state_.buttons.right_stick = get_button(SDL_CONTROLLER_BUTTON_RIGHTSTICK);
    state_.buttons.left_shoulder = get_button(SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
    state_.buttons.right_shoulder = get_button(SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
    state_.buttons.max = get_button(SDL_CONTROLLER_BUTTON_MAX);

    return state_;
}

int SDLGamepad::get_unique_id(const SDLGUID& guid) {
    static std::vector<SDLGUID> seen;

    const auto it = std::find(seen.begin(), seen.end(), guid);
    if (it != seen.end()) {
        return std::distance(seen.begin(), it);
    }

    seen.emplace_back(guid);
    return seen.size();
}

std::string SDLGamepad::to_string() const {
    std::stringstream ss;
    ss << name << " (" << guid << ") - " << instance_id << " - " << unique_id;
    return ss.str();
}

std::ostream& operator<<(std::ostream& stream, const SDLGamepad& gamepad) {
    stream << gamepad.to_string();
    return stream;
}
}  // namespace joystick
