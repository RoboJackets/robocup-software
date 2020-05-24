#pragma once

#include <Configuration.hpp>

/**
 * @brief Configuration per robot model
 */
class RobotConfig {
public:
    RobotConfig() {}
    RobotConfig(Configuration* config, const QString& prefix);

    struct PID {
        PID() {}
        PID(Configuration* config, const QString& prefix);

        std::unique_ptr<ConfigDouble> p;
        std::unique_ptr<ConfigDouble> i;

        /// how many past errors to store.  -1 means store all
        std::unique_ptr<ConfigInt> i_windup;

        std::unique_ptr<ConfigDouble> d;
    };

    struct Kicker {
        Kicker() {}
        Kicker(Configuration* config, const QString& prefix);

        /// these limits are applied before sending the actual commands to the
        /// robots
        std::unique_ptr<ConfigDouble> maxKick;
        std::unique_ptr<ConfigDouble> maxChip;
        // ConfigDouble *passKick;
    };

    struct Dribbler {
        Dribbler() {}
        Dribbler(Configuration* config, const QString& prefix);

        /// dribber values are multiplied by this before being sent to the robot
        /// this was added because 2011 bots needed lower dribbler values than
        /// the 2008 model
        std::unique_ptr<ConfigDouble> multiplier;
    };

    struct Chipper {
        Chipper() {}
        Chipper(Configuration* config, const QString& prefix);

        std::unique_ptr<ConfigDouble> calibrationSlope;
        std::unique_ptr<ConfigDouble> calibrationOffset;
    };

    PID translation;
    PID rotation;

    Kicker kicker;
    Dribbler dribbler;
    Chipper chipper;

    /// convert from real units to bot "units"
    std::unique_ptr<ConfigDouble> velMultiplier;
    std::unique_ptr<ConfigDouble> angleVelMultiplier;

    // If a command velocity we're about to send is below this value in
    // magniude, but greater than zero, we scale the velocity up to this
    // magnitude.
    std::unique_ptr<ConfigDouble> minEffectiveVelocity;
    std::unique_ptr<ConfigDouble> minEffectiveAngularSpeed;

    /// we multiply this by the bot's acceleration and add this to the output
    /// targetVel
    std::unique_ptr<ConfigDouble> accelerationMultiplier;

    // when pivoting, we multiply the calculated x-velocity of the robot by this
    // value before sending it to the robot
    std::unique_ptr<ConfigDouble> pivotVelMultiplier;
};

/**
 * Provides per-robot overrides for a robot
 * Should be updated for hardware revision
 */
class RobotLocalConfig {
public:
    RobotLocalConfig() = default;
    RobotLocalConfig(Configuration* config, const QString& prefix);

    RobotLocalConfig(RobotLocalConfig&&) = default;
    RobotLocalConfig& operator=(RobotLocalConfig&&) = default;

    std::unique_ptr<ConfigBool> chipper_enabled;
    std::unique_ptr<ConfigBool> kicker_enabled;
    std::unique_ptr<ConfigBool> ball_sense_enabled;
    std::unique_ptr<ConfigBool> dribbler_enabled;
};
