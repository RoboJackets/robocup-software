#include <joystick/SDLGamepad.hpp>
#include <iostream>

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
}

SDLGamepad::~SDLGamepad() {
    if (controller_ != nullptr) {
        std::cout << "Removing " << *this << std::endl;
        SDL_GameControllerClose(controller_);
        controller_ = nullptr;
    }
}

std::string SDLGamepad::toString() const {
    std::stringstream ss;
    ss << name << " (" << guid << ") - " << instance_id;
    return ss.str();
}

std::ostream& operator<<(std::ostream& stream, const SDLGamepad& gamepad) {
    stream << gamepad.toString();
    return stream;
}
}  // namespace joystick
