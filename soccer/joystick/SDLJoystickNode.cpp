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

void SDLJoystickNode::queryAndUpdateGamepadList() {
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
    callConnectFns(new_joystick->unique_id);
    gamepads_.emplace_back(std::move(new_joystick));
}

void SDLJoystickNode::removeJoystick(int instance_id) {
    const auto is_instance =
        [instance_id](const std::unique_ptr<SDLGamepad>& gamepad) -> bool {
        return gamepad->instance_id == instance_id;
    };

    std::optional<std::reference_wrapper<const SDLGamepad>> gamepad;
    gamepad = getGamepadByInstanceID(instance_id);
    if (gamepad) {
        callDisconnectFns(gamepad->get().unique_id);

        gamepads_.erase(
            std::remove_if(gamepads_.begin(), gamepads_.end(), is_instance),
            gamepads_.end());
    }
}

std::optional<std::reference_wrapper<const SDLGamepad>>
SDLJoystickNode::getGamepadByInstanceID(int instance_id) const {
    const auto is_instance =
        [instance_id](const std::unique_ptr<SDLGamepad>& gamepad) -> bool {
        return gamepad->instance_id == instance_id;
    };

    const auto it =
        std::find_if(gamepads_.begin(), gamepads_.end(), is_instance);

    if (it == gamepads_.end()) {
        return std::nullopt;
    }

    return std::cref(**it);
}

void SDLJoystickNode::addCallbacks(
    const GamepadCallbackFn& callback, const GamepadConnectedFn& on_connected,
    const GamepadDisconnectedFn& on_disconnected) {
    callback_fns_.emplace_back(callback);
    on_connected_fns_.emplace_back(on_connected);
    on_disconnected_fns_.emplace_back(on_disconnected);
}

void SDLJoystickNode::callCallbacks() {
    for (const auto& callback_fn : callback_fns_) {
        for (auto& gamepad : gamepads_) {
            callback_fn(gamepad->update());
        }
    }
}

void SDLJoystickNode::callDisconnectFns(int unique_id) const {
    for (const auto& callback_fn : on_disconnected_fns_) {
        callback_fn(unique_id);
    }
};

void SDLJoystickNode::callConnectFns(int unique_id) const {
    for (const auto& callback_fn : on_connected_fns_) {
        callback_fn(unique_id);
    }
}

void SDLJoystickNode::run() {
    queryAndUpdateGamepadList();
    callCallbacks();
}
}  // namespace joystick
