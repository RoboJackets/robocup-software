#pragma once

#include <SDL.h>

#include <context.hpp>
#include <node.hpp>
#include <functional>
#include <joystick/gamepad_message.hpp>
#include <joystick/sdl_gamepad.hpp>
#include <vector>

namespace joystick {
constexpr auto kGameControllerDbPath = "gamecontrollerdb.txt";
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
    void query_and_update_gamepad_list();

    /**
     * Add a joystick with the specified device id
     * @param device_index
     */
    void add_joystick(int device_index);

    /**
     * Calls update on each gamepad and puts all
     * messages into a vector (for now this lives in context)
     */
    void update_gamepad_messages();

    /**
     * Remove a joystick with the specified instance_id
     * @param instance_id
     */
    void remove_joystick(int instance_id);

    [[nodiscard]] std::optional<std::reference_wrapper<const SDLGamepad>>
    get_gamepad_by_instance_id(int instance_id) const;

    std::vector<std::unique_ptr<SDLGamepad>> gamepads_;
    Context* context_;
};
}  // namespace joystick
