#include "sdl_manual_control.hpp"

namespace joystick {

KeyboardController::KeyboardController() {
    SDL_RenderClear(renderer_);
    SDL_RenderPresent(renderer_);

    window_ = SDL_CreateWindow("Virtual joystick", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                               300, 300, 0);

    renderer_ = SDL_CreateRenderer(window_, -1, 0);
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
}

static bool key_down(const uint8_t* keystate, SDL_Scancode code) {
    return keystate[code] != 0u;  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

ControllerCommand KeyboardController::get_command() const {
    ControllerCommand command;

    const auto* keystate = SDL_GetKeyboardState(nullptr);
    if (key_down(keystate, SDL_SCANCODE_W)) {
        command.translation.y() = 1.0;
    }
    if (key_down(keystate, SDL_SCANCODE_S)) {
        command.translation.y() = -1.0;
    }
    if (key_down(keystate, SDL_SCANCODE_A)) {
        command.translation.x() = -1.0;
    }
    if (key_down(keystate, SDL_SCANCODE_D)) {
        command.translation.x() = 1.0;
    }
    if (key_down(keystate, SDL_SCANCODE_Q)) {
        command.rotation = 1;
    }
    if (key_down(keystate, SDL_SCANCODE_E)) {
        command.rotation = -1;
    }

    if (key_down(keystate, SDL_SCANCODE_LSHIFT)) {
        command.rotation *= PARAM_max_damped_rotation_speed;
        command.translation *= PARAM_max_damped_translation_speed;
    } else {
        command.rotation *= PARAM_max_rotation_speed;
        command.translation *= PARAM_max_translation_speed;
    }

    if (key_down(keystate, SDL_SCANCODE_J)) {
        command.chip = true;
        command.kick_power = kick_power_;
    } else if (key_down(keystate, SDL_SCANCODE_K)) {
        command.kick = true;
        command.kick_power = kick_power_;
    }

    if (key_down(keystate, SDL_SCANCODE_SPACE) || key_down(keystate, SDL_SCANCODE_RSHIFT)) {
        command.dribble_power = 255.0;  // max is 255
    } else {
        command.dribble_power = dribble_power_;
    }

    return command;
}

void KeyboardController::process_sdl(SDL_Event* event) {
    SDL_RenderClear(renderer_);
    SDL_RenderPresent(renderer_);

    if (event->type == SDL_EventType::SDL_KEYDOWN) {
        if (event->key.keysym.sym == SDLK_RIGHT) {
            kick_power_ += PARAM_kick_power_increment;
        } else if (event->key.keysym.sym == SDLK_LEFT) {
            kick_power_ -= PARAM_kick_power_increment;
        }
        kick_power_ = std::clamp(kick_power_, 0.0, 1.0);

        if (event->key.keysym.sym == SDLK_UP) {
            dribble_power_ += PARAM_dribble_power_increment;
        } else if (event->key.keysym.sym == SDLK_DOWN) {
            dribble_power_ -= PARAM_dribble_power_increment;
        }
        dribble_power_ = std::clamp(dribble_power_, 0.0, 1.0);
    }
}

KeyboardController::~KeyboardController() {
    SDL_DestroyRenderer(renderer_);
    SDL_DestroyWindow(window_);
}

ControllerCommand GamepadController::get_command() const {
    ControllerCommand command;
    command.translation.x() = get_axis(SDL_CONTROLLER_AXIS_LEFTY);
    command.translation.y() = -get_axis(SDL_CONTROLLER_AXIS_LEFTX);
    command.rotation = -get_axis(SDL_CONTROLLER_AXIS_RIGHTX);
    command.kick = get_axis(SDL_CONTROLLER_AXIS_TRIGGERRIGHT) > kTriggerCutoff;
    command.chip = get_button(SDL_CONTROLLER_BUTTON_LEFTSHOULDER);

    if (command.kick || command.chip) {
        command.kick_power = kick_power_;
    }

    command.dribble_power = dribble_power_;
    return command;
}

void GamepadController::process_sdl(SDL_Event* event) {
    if (event->type == SDL_EventType::SDL_CONTROLLERBUTTONDOWN &&
        event->cbutton.which == get_id()) {
        if (event->cbutton.button == SDL_CONTROLLER_BUTTON_DPAD_RIGHT) {
            kick_power_ += PARAM_kick_power_increment;
        } else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_DPAD_LEFT) {
            kick_power_ -= PARAM_kick_power_increment;
        }
        kick_power_ = std::clamp(kick_power_, 0.0, 1.0);

        if (event->cbutton.button == SDL_CONTROLLER_BUTTON_DPAD_UP) {
            dribble_power_ += PARAM_dribble_power_increment;
        } else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_DPAD_DOWN) {
            dribble_power_ -= PARAM_dribble_power_increment;
        }
        dribble_power_ = std::clamp(dribble_power_, 0.0, 1.0);
    }
}

SDLControllerProvider::~SDLControllerProvider() { SDL_Quit(); }

SDLControllerProvider::SDLControllerProvider(bool do_keyboard,
                                             std::function<void(ManualController*)> on_connect,
                                             std::function<void(ManualController*)> on_disconnect)
    : on_connect_{std::move(on_connect)}, on_disconnect_{std::move(on_disconnect)} {
    // initialize using the SDL joystick
    if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS | SDL_INIT_VIDEO) != 0) {
        FATAL_THROW("SDL could not initialize game controller system! SDL Error: {}",
                    SDL_GetError());
    }

    // Attempt to add additional mappings (relative to run)
    const auto share_dir = ament_index_cpp::get_package_share_directory("rj_robocup");
    std::stringstream sdl_path;
    sdl_path << share_dir << "/gamecontrollerdb.txt";
    if (SDL_GameControllerAddMappingsFromFile(sdl_path.str().c_str()) == -1) {
        SPDLOG_ERROR("Failed adding additional SDL Gamecontroller Mappings: {}", SDL_GetError());
    }

    if (do_keyboard) {
        auto keyboard_controller = std::make_unique<KeyboardController>();
        on_connect_(keyboard_controller.get());
        controllers_.emplace_back(std::move(keyboard_controller));
    }
}

void SDLControllerProvider::update() {
    SDL_Event event;
    while (SDL_PollEvent(&event) != 0) {
        if (event.type == SDL_EventType::SDL_CONTROLLERDEVICEADDED) {
            SDL_GameController* controller = SDL_GameControllerFromInstanceID(event.cdevice.which);
            controllers_.push_back(std::make_unique<GamepadController>(controller));
            on_connect_(controllers_.back().get());
        } else if (event.type == SDL_EventType::SDL_CONTROLLERDEVICEREMOVED) {
            auto it = std::find_if(controllers_.begin(), controllers_.end(),
                                   [which = event.cdevice.which](const auto& controller) {
                                       return controller->get_id() == which;
                                   });
            if (it != controllers_.end()) {
                on_disconnect_(it->get());
                controllers_.erase(it);
            } else {
                SPDLOG_WARN("Controller {} disconnected, but it does not exist.",
                            event.cdevice.which);
            }
        } else {
            for (auto& controller : controllers_) {
                controller->process_sdl(&event);
            }
        }
    }
}

}  // namespace joystick
