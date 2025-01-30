#pragma once

#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rj_geometry/point.hpp>
#include <rj_param_utils/param.hpp>

namespace joystick {

constexpr auto kJoystickModule = "joystick";

DECLARE_BOOL(kJoystickModule, use_field_oriented_drive)
DECLARE_BOOL(kJoystickModule, kick_on_break_beam)
DECLARE_BOOL(kJoystickModule, damped_translation)
DECLARE_BOOL(kJoystickModule, damped_rotation)
DECLARE_FLOAT64(kJoystickModule, max_rotation_speed)
DECLARE_FLOAT64(kJoystickModule, max_damped_rotation_speed)
DECLARE_FLOAT64(kJoystickModule, max_translation_speed)
DECLARE_FLOAT64(kJoystickModule, max_damped_translation_speed)
DECLARE_FLOAT64(kJoystickModule, kick_power_increment)
DECLARE_FLOAT64(kJoystickModule, dribble_power_increment)

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
