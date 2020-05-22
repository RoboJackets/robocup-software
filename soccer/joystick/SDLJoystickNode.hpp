#pragma once

#include <SDL.h>

#include <Context.hpp>
#include <Node.hpp>
#include <functional>
#include <joystick/GamepadMessage.hpp>
#include <joystick/SDLGamepad.hpp>
#include <vector>

namespace joystick {
constexpr auto GameControllerDBPath = "gamecontrollerdb.txt";
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
