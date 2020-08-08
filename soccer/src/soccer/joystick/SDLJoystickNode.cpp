#include "SDLJoystickNode.hpp"

#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace joystick {
SDLJoystickNode::SDLJoystickNode(Context* context) : context_{context} {
    // initialize using the SDL joystick
    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS) != 0) {
        std::cerr << "ERROR: SDL could not initialize game controller system! SDL "
                     "Error: "
                  << SDL_GetError() << std::endl;
        throw std::runtime_error("");
    }

    // Attempt to add additional mappings (relative to run)
    const auto share_dir = ament_index_cpp::get_package_share_directory("rj_robocup");
    std::stringstream sdl_path;
    sdl_path << share_dir << "/gamecontrollerdb.txt";
    if (SDL_GameControllerAddMappingsFromFile(sdl_path.str().c_str()) == -1) {
        std::cout << "Failed adding additional SDL Gamecontroller Mappings: " << SDL_GetError()
                  << std::endl;
    }
}

SDLJoystickNode::~SDLJoystickNode() { SDL_Quit(); }

void SDLJoystickNode::query_and_update_gamepad_list() {
    SDL_GameControllerUpdate();
    SDL_Event event;
    while (SDL_PollEvent(&event) == 1) {
        switch (event.type) {
            case SDL_CONTROLLERDEVICEADDED:
                add_joystick(event.cdevice.which);
                break;
            case SDL_CONTROLLERDEVICEREMOVED:
                remove_joystick(event.cdevice.which);
                break;
        }
    }
}

void SDLJoystickNode::add_joystick(int device_index) {
    auto new_joystick = std::make_unique<SDLGamepad>(device_index);
    context_->gamepads.emplace_back(new_joystick->unique_id);
    gamepads_.emplace_back(std::move(new_joystick));
}

void SDLJoystickNode::remove_joystick(int instance_id) {
    const auto is_instance = [instance_id](const std::unique_ptr<SDLGamepad>& gamepad) -> bool {
        return gamepad->instance_id == instance_id;
    };

    std::optional<std::reference_wrapper<const SDLGamepad>> gamepad;
    gamepad = get_gamepad_by_instance_id(instance_id);
    if (gamepad) {
        const int unique_id = gamepad->get().unique_id;
        // Remove gamepad from context gamepads
        std::vector<int>& gamepad_stack = context_->gamepads;
        gamepad_stack.erase(std::remove(gamepad_stack.begin(), gamepad_stack.end(), unique_id),
                            gamepad_stack.end());

        // Remove own list
        gamepads_.erase(std::remove_if(gamepads_.begin(), gamepads_.end(), is_instance),
                        gamepads_.end());
    }
}

std::optional<std::reference_wrapper<const SDLGamepad>> SDLJoystickNode::get_gamepad_by_instance_id(
    int instance_id) const {
    const auto is_instance = [instance_id](const std::unique_ptr<SDLGamepad>& gamepad) -> bool {
        return gamepad->instance_id == instance_id;
    };

    const auto it = std::find_if(gamepads_.begin(), gamepads_.end(), is_instance);

    if (it == gamepads_.end()) {
        return std::nullopt;
    }

    return std::cref(**it);
}

void SDLJoystickNode::update_gamepad_messages() {
    context_->gamepad_messages.clear();
    for (auto& gamepad : gamepads_) {
        context_->gamepad_messages.emplace_back(gamepad->update());
    }
}

void SDLJoystickNode::run() {
    query_and_update_gamepad_list();
    update_gamepad_messages();
}
}  // namespace joystick
