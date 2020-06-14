
// FIXME - Move a lot of stuff like blueTeam and worldToTeam to a globally
// accessible place

#pragma once

#include <protobuf/LogFrame.pb.h>

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Pose.hpp>
#include <Geometry2d/TransformMatrix.hpp>
#include <Logger.hpp>
#include <Referee.hpp>
#include <SystemState.hpp>
#include <mutex>
#include <optional>
#include <vector>

#include "GrSimCommunicator.hpp"
#include "Node.hpp"
#include "VisionReceiver.hpp"
#include "joystick/ManualControlNode.hpp"
#include "joystick/SDLJoystickNode.hpp"
#include "motion/MotionControlNode.hpp"
#include "planning/PlannerNode.hpp"
#include "radio/Radio.hpp"
#include "radio/RadioNode.hpp"
#include "rc-fshare/rtp.hpp"

class Configuration;
class RobotLocalConfig;
class Joystick;
struct JoystickControlValues;
class Radio;
class VisionFilter;

namespace Gameplay {
class GameplayModule;
}

namespace Planning {
class MultiRobotPathPlanner;
}

/**
 * @brief Brings all the pieces together
 *
 * @details The processor ties together all the moving parts for controlling
 * a team of soccer robots.  Its responsibities include:
 * - receiving and handling vision packets (see VisionReceiver)
 * - receiving and handling referee packets (see RefereeModule)
 * - radio IO (see Radio)
 * - running the BallTracker
 * - running the Gameplay::GameplayModule
 * - running the Logger
 * - handling the Configuration
 * - handling the Joystick
 * - running motion control for each robot (see OurRobot#motionControl)
 */
class Processor {
public:
    struct Status {
        Status() {}

        RJ::Time lastLoopTime;
        RJ::Time lastVisionTime;
        RJ::Time lastRefereeTime;
        RJ::Time lastRadioRxTime;
    };

    static void createConfiguration(Configuration* cfg);

    Processor(bool sim, bool blueTeam, const std::string& readLogFile = "");
    virtual ~Processor();

    void stop();

    std::shared_ptr<Gameplay::GameplayModule> gameplayModule() const {
        return _gameplayModule;
    }

    std::shared_ptr<Referee> refereeModule() const { return _refereeModule; }

    SystemState* state() { return &_context.state; }

    Status status() {
        std::lock_guard lock(_statusMutex);
        return _status;
    }

    float framerate() { return _framerate; }

    bool openLog(const QString& filename) {
        _logger->write(filename.toStdString());
        return true;
    }

    void closeLog() { _logger->close(); }

    std::lock_guard<std::mutex> lockLoopMutex() {
        return std::lock_guard(_loopMutex);
    }

    std::mutex* loopMutex() { return &_loopMutex; }

    Radio* radio() { return _radio->getRadio(); }

    /**
     * Stops all robots by clearing their intents and setpoints
     */
    void stopRobots();

    void recalculateWorldToTeamTransform();

    void setFieldDimensions(const Field_Dimensions& dims);

    bool isRadioOpen() const;

    bool isInitialized() const;

    Context* context() { return &_context; }

    void run();

private:
    // Configuration for the robot.
    // TODO(Kyle): Add back in configuration values for different years.
    static std::unique_ptr<RobotConfig> robot_config_init;

    // per-robot status configs
    static std::vector<RobotLocalConfig*> robotStatuses;

    /**
     * Updates the intent.active for each robot.
     *
     * The intent is active if it's being joystick controlled or
     * if it's visible
     */
    void updateIntentActive();

    void updateGeometryPacket(const SSL_GeometryFieldSize& fieldSize);

    void updateOrientation();

    void runModels();

    /** Used to start and stop the thread **/
    volatile bool _running;

    // A logfile to read from.
    // When empty, don't read logs at all.
    std::string _readLogFile;

    // Locked when processing loop stuff is happening (not when blocked for
    // timing or I/O). This is public so the GUI thread can lock it to access
    // SystemState, etc.
    std::mutex _loopMutex;

    /** global system state */
    Context _context;

    // Transformation from world space to team space.
    // This depends on which goal we're defending.
    //
    // _teamTrans is used for positions, not angles.
    // _teamAngle is used for angles.
    Geometry2d::TransformMatrix _worldToTeam;
    float _teamAngle{};

    // Processing period in microseconds
    RJ::Seconds _framePeriod = RJ::Seconds(1) / 60;

    /// Measured framerate
    float _framerate;

    // This is used by the GUI to indicate status of the processing loop and
    // network
    std::mutex _statusMutex;
    Status _status;

    // modules
    std::shared_ptr<VisionFilter> _vision;
    std::shared_ptr<Referee> _refereeModule;
    std::shared_ptr<Gameplay::GameplayModule> _gameplayModule;
    std::unique_ptr<VisionReceiver> _visionReceiver;
    std::unique_ptr<MotionControlNode> _motionControl;
    std::unique_ptr<Planning::PlannerNode> _planner_node;
    std::unique_ptr<RadioNode> _radio;
    std::unique_ptr<GrSimCommunicator> _grSimCom;
    std::unique_ptr<joystick::SDLJoystickNode> _sdl_joystick_node;
    std::unique_ptr<joystick::ManualControlNode> _manual_control_node;
    std::unique_ptr<Logger> _logger;

    std::vector<Node*> _nodes;

    bool _initialized;
};
