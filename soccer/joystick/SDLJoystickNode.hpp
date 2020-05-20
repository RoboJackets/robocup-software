#pragma once

#include <SDL.h>

#include <Node.hpp>
#include <joystick/JoystickMsg.hpp>
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

private:
    /**
     * Queries for joysticks, and updates the current
     * list of connected joysticks (inserting + deleting)
     */
    void queryAndUpdateJoystickList();

    void addJoystick(int device_index);
    void removeJoystick(int instance_id);
    std::vector<std::unique_ptr<SDLGamepad>> gamepads_;
};
}  // namespace joystick
