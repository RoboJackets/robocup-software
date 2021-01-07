#pragma once

#include <optional>

#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/pose.hpp>

/**
 * \brief Struct representing data returned from the robot, possibly including
 * state updates from an on-board Kalman filter.
 */
struct RobotStatus {
    RJ::Time timestamp;

    RobotId shell_id = 0;

    enum class HardwareVersion { kUnknown, kFleet2018, kSimulated };

    HardwareVersion version = HardwareVersion::kFleet2018;

    std::optional<rj_geometry::Twist> twist_estimate;
    std::optional<rj_geometry::Pose> pose_estimate;

    double battery_voltage = 0.0;
    double kicker_voltage = 0.0;

    bool has_ball = false;

    enum class KickerState { kFailed, kCharging, kCharged };

    KickerState kicker = KickerState::kFailed;

    std::array<bool, 5> motors_healthy{};

    bool fpga_healthy = false;
};
