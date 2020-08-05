#include <algorithm>
#include <iostream>
#include <vector>

#include <joystick/SDLGamepad.hpp>

namespace joystick {
SDLGamepad::SDLGamepad(int device_index) {
    controller_ = SDL_GameControllerOpen(device_index);

    if (controller_ == nullptr) {
        std::cerr << "ERROR: Could not open controller! SDL Error: "
                  << SDL_GetError() << std::endl;
        throw std::runtime_error("Failed to open controller!");
    }

    auto* joystick = SDL_GameControllerGetJoystick(controller_);

    name = std::string{SDL_JoystickName(joystick)};
    instance_id = SDL_JoystickInstanceID(joystick);
    guid = SDL_JoystickGetGUID(joystick);
    unique_id = SDLGamepad::getUniqueID(guid);

    state_.unique_id = unique_id;
}

SDLGamepad::~SDLGamepad() {
    if (controller_ != nullptr) {
        SDL_GameControllerClose(controller_);
        controller_ = nullptr;
    }
}

bool SDLGamepad::getButton(SDL_GameControllerButton button) {
    return static_cast<bool>(SDL_GameControllerGetButton(controller_, button));
}

int32_t SDLGamepad::getAxis(SDL_GameControllerAxis axis) {
    return SDL_GameControllerGetAxis(controller_, axis);
}

const GamepadMessage& SDLGamepad::update() {
    state_.update_time = RJ::now();

    // Sticks
    state_.sticks.left.x = getAxis(SDL_CONTROLLER_AXIS_LEFTX);
    state_.sticks.left.y = getAxis(SDL_CONTROLLER_AXIS_LEFTY);
    state_.sticks.right.x = getAxis(SDL_CONTROLLER_AXIS_RIGHTX);
    state_.sticks.right.y = getAxis(SDL_CONTROLLER_AXIS_RIGHTY);

    // Triggers
    state_.triggers.left = getAxis(SDL_CONTROLLER_AXIS_TRIGGERLEFT);
    state_.triggers.right = getAxis(SDL_CONTROLLER_AXIS_TRIGGERRIGHT);

    // DPAD
    state_.dpad.up = getButton(SDL_CONTROLLER_BUTTON_DPAD_UP);
    state_.dpad.down = getButton(SDL_CONTROLLER_BUTTON_DPAD_DOWN);
    state_.dpad.left = getButton(SDL_CONTROLLER_BUTTON_DPAD_LEFT);
    state_.dpad.right = getButton(SDL_CONTROLLER_BUTTON_DPAD_RIGHT);

    // Buttons
    state_.buttons.a = getButton(SDL_CONTROLLER_BUTTON_A);
    state_.buttons.b = getButton(SDL_CONTROLLER_BUTTON_B);
    state_.buttons.x = getButton(SDL_CONTROLLER_BUTTON_X);
    state_.buttons.y = getButton(SDL_CONTROLLER_BUTTON_Y);
    state_.buttons.back = getButton(SDL_CONTROLLER_BUTTON_BACK);
    state_.buttons.guide = getButton(SDL_CONTROLLER_BUTTON_GUIDE);
    state_.buttons.start = getButton(SDL_CONTROLLER_BUTTON_START);
    state_.buttons.left_stick = getButton(SDL_CONTROLLER_BUTTON_LEFTSTICK);
    state_.buttons.right_stick = getButton(SDL_CONTROLLER_BUTTON_RIGHTSTICK);
    state_.buttons.left_shoulder =
        getButton(SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
    state_.buttons.right_shoulder =
        getButton(SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
    state_.buttons.max = getButton(SDL_CONTROLLER_BUTTON_MAX);

    return state_;
}

int SDLGamepad::getUniqueID(const SDLGUID& guid) {
    static std::vector<SDLGUID> seen;

    const auto it = std::find(seen.begin(), seen.end(), guid);
    if (it != seen.end()) {
        return std::distance(seen.begin(), it);
    }

    seen.emplace_back(guid);
    return seen.size();
}

std::string SDLGamepad::toString() const {
    std::stringstream ss;
    ss << name << " (" << guid << ") - " << instance_id << " - " << unique_id;
    return ss.str();
}

std::ostream& operator<<(std::ostream& stream, const SDLGamepad& gamepad) {
    stream << gamepad.toString();
    return stream;
}
}  // namespace joystick
