#include "SDLJoystickNode.hpp"

#include <Utils.hpp>
#include <iostream>

namespace joystick {
SDLJoystickNode::SDLJoystickNode() {
    // initialize using the SDL joystick
    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS) != 0) {
        std::cerr
            << "ERROR: SDL could not initialize game controller system! SDL "
               "Error: "
            << SDL_GetError() << std::endl;
        throw std::runtime_error("");
    }

    // Attempt to add additional mappings (relative to run)
    if (SDL_GameControllerAddMappingsFromFile(
            ApplicationRunDirectory()
                .filePath(GameControllerDBPath)
                .toStdString()
                .c_str()) == -1) {
        std::cout << "Failed adding additional SDL Gamecontroller Mappings: "
                  << SDL_GetError() << std::endl;
    }
}

SDLJoystickNode::~SDLJoystickNode() { SDL_Quit(); }

void SDLJoystickNode::queryAndUpdateJoystickList() {
    SDL_GameControllerUpdate();
    SDL_Event event;
    while (SDL_PollEvent(&event) == 1) {
        switch (event.type) {
            case SDL_CONTROLLERDEVICEADDED:
                addJoystick(event.cdevice.which);
                break;
            case SDL_CONTROLLERDEVICEREMOVED:
                removeJoystick(event.cdevice.which);
                break;
        }
    }
}

void SDLJoystickNode::addJoystick(int device_index) {
    auto new_joystick = std::make_unique<SDLGamepad>(device_index);
    gamepads_.emplace_back(std::move(new_joystick));
    std::cout << "Added " << *gamepads_.back() << std::endl;
}

void SDLJoystickNode::removeJoystick(int instance_id) {
    const auto is_instance = [instance_id](const std::unique_ptr<SDLGamepad>& gamepad) -> bool {
        return gamepad->instance_id == instance_id;
    };

    gamepads_.erase(
        std::remove_if(gamepads_.begin(), gamepads_.end(), is_instance),
        gamepads_.end());
}

void SDLJoystickNode::run() { queryAndUpdateJoystickList(); }
}  // namespace joystick
