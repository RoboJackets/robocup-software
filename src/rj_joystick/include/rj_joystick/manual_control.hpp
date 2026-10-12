#pragma once

#include <string>

#include <rj_geometry/point.hpp>

namespace joystick {

// Read once at startup from the /manual_control section of the params YAML
// (see ManualControlNode).
struct ManualControlParams {
    double max_rotation_speed;            // rad/s
    double max_damped_rotation_speed;     // rad/s
    double max_translation_speed;         // m/s
    double max_damped_translation_speed;  // m/s
    double kick_power_increment;          // 0-1
    double dribble_power_increment;       // 0-1
    bool kick_on_break_beam;
    double min_kick_speed;  // m/s
    double max_kick_speed;  // m/s
};

struct ControllerCommand {
    rj_geometry::Point translation;
    double rotation = 0;
    bool kick = false;
    bool chip = false;
    double kick_power = 0;
    double dribble_power = 0;
};

class ManualController {
public:
    [[nodiscard]] virtual ControllerCommand get_command() const = 0;
    [[nodiscard]] virtual std::string get_description() const = 0;
    [[nodiscard]] virtual std::string get_uuid() const = 0;
};

class ManualControllerProvider {
public:
    ManualControllerProvider() = default;
    virtual ~ManualControllerProvider() = default;

    ManualControllerProvider(const ManualControllerProvider&) = delete;
    ManualControllerProvider& operator=(const ManualControllerProvider&) = delete;
    ManualControllerProvider(ManualControllerProvider&&) = delete;
    ManualControllerProvider& operator=(const ManualControllerProvider&&) = delete;

    virtual void update() = 0;
};

}  // namespace joystick
