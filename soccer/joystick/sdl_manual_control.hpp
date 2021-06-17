#pragma once

#include <SDL.h>
#include <fmt/format.h>

#include <rj_utils/logging.hpp>

#include "manual_control.hpp"

namespace joystick {

static constexpr float kTriggerCutoff = 0.9;

class SDLController : public ManualController {
public:
    SDLController() = default;
    virtual ~SDLController() = default;

    SDLController(const SDLController&) = delete;
    SDLController& operator=(const SDLController&) = delete;
    SDLController(SDLController&&) = delete;
    SDLController& operator=(const SDLController&&) = delete;

    virtual void process_sdl(SDL_Event* event) = 0;
    [[nodiscard]] virtual SDL_JoystickID get_id() const = 0;
};

class KeyboardController : public SDLController {
public:
    KeyboardController();
    ~KeyboardController() override;

    KeyboardController(const KeyboardController&) = delete;
    KeyboardController& operator=(const KeyboardController&) = delete;
    KeyboardController(KeyboardController&&) = delete;
    KeyboardController& operator=(const KeyboardController&&) = delete;

    [[nodiscard]] ControllerCommand get_command() const override;

    [[nodiscard]] std::string get_description() const override { return "Keyboard controller"; }

    [[nodiscard]] std::string get_uuid() const override { return "keyboard-controller"; }

    void process_sdl(SDL_Event* event) override;

    [[nodiscard]] SDL_JoystickID get_id() const override { return -1; }

private:
    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;

    double kick_power_ = 0.5;
    double dribble_power_ = 0.0;
};

class GamepadController : public SDLController {
public:
    GamepadController(SDL_GameController* controller) : my_controller_{controller} {}
    ~GamepadController() override = default;

    GamepadController(const GamepadController&) = delete;
    GamepadController& operator=(const GamepadController&) = delete;
    GamepadController(GamepadController&&) = delete;
    GamepadController& operator=(const GamepadController&&) = delete;

    [[nodiscard]] ControllerCommand get_command() const override;

    [[nodiscard]] std::string get_description() const override {
        return fmt::format("Gamepad controller {}", get_id());
    }

    [[nodiscard]] std::string get_uuid() const override {
        return fmt::format("gamepad-controller-{}", get_id());
    }

    void process_sdl(SDL_Event* event) override;

    [[nodiscard]] SDL_JoystickID get_id() const override {
        return SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(my_controller_));
    }

private:
    [[nodiscard]] bool get_button(SDL_GameControllerButton button) const {
        return static_cast<bool>(SDL_GameControllerGetButton(my_controller_, button));
    }

    [[nodiscard]] float get_axis(SDL_GameControllerAxis axis) const {
        static constexpr float kAxisMax = 32768.0f;
        return static_cast<float>(SDL_GameControllerGetAxis(my_controller_, axis)) / kAxisMax;
    }

    SDL_GameController* my_controller_;

    double kick_power_ = 0.5;
    double dribble_power_ = 0.0;
};

/**
 * Handles registering and deregistering SDL input devices (gamepads) when necessary.
 */
class SDLControllerProvider : public ManualControllerProvider {
public:
    SDLControllerProvider(bool do_keyboard, std::function<void(ManualController*)> on_connect,
                          std::function<void(ManualController*)> on_disconnect);
    ~SDLControllerProvider() override;

    SDLControllerProvider(const SDLControllerProvider&) = delete;
    SDLControllerProvider& operator=(const SDLControllerProvider&) = delete;
    SDLControllerProvider(SDLControllerProvider&&) = delete;
    SDLControllerProvider& operator=(const SDLControllerProvider&&) = delete;

    void update() override;

private:
    std::vector<std::unique_ptr<SDLController>> controllers_;
    std::function<void(ManualController*)> on_connect_;
    std::function<void(ManualController*)> on_disconnect_;
};

}  // namespace joystick
