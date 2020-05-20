#pragma once

#include <SDL.h>
#include <string>
#include <sstream>
#include <array>
#include <iomanip>

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

    friend std::ostream& operator<<(std::ostream& stream, const SDLGamepad& gamepad);

    std::string name;
    int instance_id;
    SDLGUID guid;

private:
    SDL_GameController* controller_;
};
}  // namespace joystick