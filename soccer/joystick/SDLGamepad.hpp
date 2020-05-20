#pragma once

#include <SDL.h>

#include <array>
#include <iomanip>
#include <sstream>
#include <string>

#include "GamepadMessage.hpp"

namespace joystick {
struct SDLGUID {
    std::array<uint8_t, 16> data{};

    SDLGUID() = default;
    SDLGUID(const SDL_JoystickGUID& guid) {
        for (int i = 0; i < 16; i++) {
            data[i] = guid.data[i];
        }
    }

    [[nodiscard]] std::string toString() const {
        std::stringstream ss;
        ss << std::hex << std::setfill('0');

        for (int i = 0; i < data.size(); i++) {
            if (i > 0) {
                ss << "-";
            }
            ss << std::setw(2) << static_cast<uint32_t>(data[i]);
        }
        return ss.str();
    }

    bool operator==(const SDLGUID& other) const { return data == other.data; }

    friend std::ostream& operator<<(std::ostream& stream, const SDLGUID& guid) {
        stream << guid.toString();
        return stream;
    }
};

class SDLGamepad {
public:
    SDLGamepad(int device_index);
    ~SDLGamepad();

    // Noncopyable
    SDLGamepad(const SDLGamepad&) = delete;
    SDLGamepad& operator=(const SDLGamepad&) = delete;
    SDLGamepad(SDLGamepad&&) = delete;
    SDLGamepad& operator=(SDLGamepad&&) = delete;

    [[nodiscard]] std::string toString() const;

    /**
     * Returns a unique ID for the passed in GUID
     * @param guid
     * @return Unique ID for the given GUID
     */
    static int getUniqueID(const SDLGUID& guid);

    bool getButton(SDL_GameControllerButton button);
    int32_t getAxis(SDL_GameControllerAxis axis);

    /**
     * Updates the internal state_ variable and returns a const ref to it
     * @return A const ref to the internal GamepadMessage
     */
    const GamepadMessage& update();

    friend std::ostream& operator<<(std::ostream& stream,
                                    const SDLGamepad& gamepad);

    /** \brief Name of the gamepad. */
    std::string name;
    /** \brief Instance ID given by libsdl. */
    int instance_id;
    /** \brief GUID assigned by libsdl. */
    SDLGUID guid;
    /** \brief Unique ID for each unique guid. Increments from 0. */
    int unique_id;

private:
    SDL_GameController* controller_;
    GamepadMessage state_;
};
}  // namespace joystick