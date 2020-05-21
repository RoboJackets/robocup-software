#pragma once

#include <SDL.h>

#include <Context.hpp>
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
    SDLJoystickNode(Context* context);

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
     * - Processing joystick input and calling callbacks
     */
    void run() override;

    using GamepadCallbackFn = std::function<void(const GamepadMessage&)>;
    using GamepadConnectedFn = std::function<void(int unique_id)>;
    using GamepadDisconnectedFn = std::function<void(int unique_id)>;

    /**
     * Add a callback function
     * @param callback Function that will get called on every connected joystick
     * each update
     */
    void addCallbacks(const GamepadCallbackFn& callback,
                      const GamepadConnectedFn& on_connected,
                      const GamepadDisconnectedFn& on_disconnected);

private:
    /**
     * Queries for joysticks, and updates the current
     * list of connected joysticks (inserting + deleting)
     */
    void queryAndUpdateGamepadList();

    /**
     * Add a joystick with the specified device id
     * @param device_index
     */
    void addJoystick(int device_index);

    /**
     * Calls update on each gamepad and puts all
     * messages into a vector (for now this lives in context)
     */
    void updateGamepadMessages();

    /**
     * Remove a joystick with the specified instance_id
     * @param instance_id
     */
    void removeJoystick(int instance_id);

    [[nodiscard]] std::optional<std::reference_wrapper<const SDLGamepad>>
    getGamepadByInstanceID(int instance_id) const;

    std::vector<std::unique_ptr<SDLGamepad>> gamepads_;
    Context* context_;
};
}  // namespace joystick
