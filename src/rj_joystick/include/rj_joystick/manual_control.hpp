#pragma once

#include <string>

#include <rj_geometry/point.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace joystick {

struct ManualControlParams {
    bool use_field_oriented_drive = false;
    bool damped_translation = false;
    bool damped_rotation = false;
    double max_rotation_speed = 4.0;
    double max_damped_rotation_speed = 1.0;
    double max_translation_speed = 2.0;
    double max_damped_translation_speed = 0.5;
    double kick_power_increment = 0.1;
    double dribble_power_increment = 0.1;
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
