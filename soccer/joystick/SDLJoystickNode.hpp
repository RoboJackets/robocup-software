#pragma once

#include <SDL.h>

#include <Node.hpp>
#include <functional>
#include <joystick/GamepadMessage.hpp>
#include <joystick/SDLGamepad.hpp>
#include <vector>

namespace joystick {
constexpr auto GameControllerDBPath =
    "../external/sdlcontrollerdb/gamecontrollerdb.txt";
/**
 * Node that uses libsdl for handling joysticks input
 */
class SDLJoystickNode : Node {
public:
    /**
     * Initializes SDL
     */
    SDLJoystickNode();

    /**
     * Cleans up SDL
     */
    ~SDLJoystickNode() override;

    // Noncopyable
    SDLJoystickNode(const SDLJoystickNode&) = delete;
    SDLJoystickNode& operator=(const SDLJoystickNode&) = delete;
    SDLJoystickNode(SDLJoystickNode&&) = delete;
    SDLJoystickNode& operator=(SDLJoystickNode&&) = delete;

    /**
     * Performs all logic, ie.
     * - Finding joysticks
     * - Processing joystick input
     */
    void run() override;

    using GamepadCallbackFn = std::function<void(const GamepadMessage&)>;
    using JoystickConnectedFn = std::function<void(int unique_id)>;
    using JoystickDisconnectedFn = std::function<void(int unique_id)>;

    /**
     * Add a callback function
     * @param callback Function that will get called on every connected joystick
     * each update
     */
    void addCallbacks(const GamepadCallbackFn& callback,
                      const JoystickConnectedFn& on_connected,
                      const JoystickDisconnectedFn& on_disconnected);

private:
    /**
     * Queries for joysticks, and updates the current
     * list of connected joysticks (inserting + deleting)
     */
    void queryAndUpdateGamepadList();

    /**
     * Get the updated states for each gamepad we have
     */
    void callCallbacks();

    /**
     * Add a joystick with the specified device id
     * @param device_index
     */
    void addJoystick(int device_index);

    /**
     * Remove a joystick with the specified instance_id
     * @param instance_id
     */
    void removeJoystick(int instance_id);

    void callDisconnectFns(int unique_id) const;
    void callConnectFns(int unique_id) const;

    [[nodiscard]] std::optional<std::reference_wrapper<const SDLGamepad>>
    getGamepadByInstanceID(int instance_id) const;

    std::vector<std::unique_ptr<SDLGamepad>> gamepads_;

    std::vector<GamepadCallbackFn> callback_fns_;
    std::vector<JoystickConnectedFn> on_connected_fns_;
    std::vector<JoystickDisconnectedFn> on_disconnected_fns_;
};
}  // namespace joystick
