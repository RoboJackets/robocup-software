
// FIXME - Move a lot of stuff like blueTeam and worldToTeam to a globally
// accessible place

#pragma once

#include <protobuf/LogFrame.pb.h>
#include <string.h>

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Pose.hpp>
#include <Geometry2d/TransformMatrix.hpp>
#include <Logger.hpp>
#include <QMutex>
#include <QMutexLocker>
#include <QThread>
#include <Referee.hpp>
#include <SystemState.hpp>
#include <optional>
#include <vector>

#include "GrSimCommunicator.hpp"
#include "Node.hpp"
#include "VisionReceiver.hpp"
#include "motion/MotionControlNode.hpp"
#include "radio/Radio.hpp"
#include "radio/RadioNode.hpp"
#include "rc-fshare/rtp.hpp"

class Configuration;
class RobotStatus;
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
class Processor : public QThread {
public:
    struct Status {
        Status() {}

        RJ::Time lastLoopTime;
        RJ::Time lastVisionTime;
        RJ::Time lastRefereeTime;
        RJ::Time lastRadioRxTime;
    };

    enum VisionChannel { primary, secondary, full };

    static void createConfiguration(Configuration* cfg);

    Processor(bool sim, bool defendPlus, VisionChannel visionChannel,
              bool blueTeam, std::string readLogFile);
    virtual ~Processor();

    void stop();

    bool autonomous();
    bool joystickValid() const;

    JoystickControlValues getJoystickControlValue(Joystick& joy);
    std::vector<JoystickControlValues> getJoystickControlValues();

    void externalReferee(bool value) {
        _refereeModule->useExternalReferee(value);
    }

    bool externalReferee() const {
        return _refereeModule->useExternalReferee();
    }

    void manualID(int value);
    int manualID() const { return _manualID; }

    void multipleManual(bool value);
    bool multipleManual() const { return _multipleManual; }

    bool useFieldOrientedManualDrive() const {
        return _useFieldOrientedManualDrive;
    }
    void setUseFieldOrientedManualDrive(bool foc) {
        _useFieldOrientedManualDrive = foc;
    }

    /**
     * @brief Set the shell ID of the goalie
     * @details The rules require us to specify at the start of a match/period
     * which
     * robot will be the goalie.  A value of -1 indicates that there is no one
     * assigned.
     */
    void goalieID(int value);
    /**
     * @brief Shell ID of the goalie robot
     */
    int goalieID();

    void dampedRotation(bool value);
    void dampedTranslation(bool value);

    void joystickKickOnBreakBeam(bool value);
    void setupJoysticks();
    std::vector<int> getJoystickRobotIds();

    void blueTeam(bool value);
    bool blueTeam() const { return _blueTeam; }

    std::shared_ptr<Gameplay::GameplayModule> gameplayModule() const {
        return _gameplayModule;
    }

    std::shared_ptr<Referee> refereeModule() const { return _refereeModule; }

    SystemState* state() { return &_context.state; }

    bool simulation() const { return _simulation; }

    void defendPlusX(bool value);
    bool defendPlusX() { return _context.game_state.defendPlusX; }

    Status status() {
        QMutexLocker lock(&_statusMutex);
        return _status;
    }

    float framerate() { return _framerate; }

    const Logger& logger() const { return _logger; }

    bool openLog(const QString& filename) { return _logger.open(filename); }

    void closeLog() { _logger.close(); }

    // Use all/part of the field
    void useOurHalf(bool value) { _useOurHalf = value; }

    void useOpponentHalf(bool value) { _useOpponentHalf = value; }

    QMutex& loopMutex() { return _loopMutex; }

    Radio* radio() { return _radio->getRadio(); }

    void changeVisionChannel(int port);

    VisionChannel visionChannel() { return _visionChannel; }

    void recalculateWorldToTeamTransform();

    void setFieldDimensions(const Field_Dimensions& dims);

    bool isRadioOpen() const;

    bool isInitialized() const;

    void setPaused(bool paused) { _paused = paused; }

    ////////

    // Time of the first LogFrame
    std::optional<RJ::Time> firstLogTime;

    Context* context() { return &_context; }

protected:
    void run() override;

    void applyJoystickControls(const JoystickControlValues& controlVals,
                               OurRobot* robot);

private:
    // Configuration for the robot.
    // TODO(Kyle): Add back in configuration values for different years.
    static std::unique_ptr<RobotConfig> robot_config_init;

    // per-robot status configs
    static std::vector<RobotStatus*> robotStatuses;

    /** send out the radio data for the radio program */
    void sendRadioData();

    void updateGeometryPacket(const SSL_GeometryFieldSize& fieldSize);

    void runModels();

    /** Used to start and stop the thread **/
    volatile bool _running;

    Logger _logger;

    bool _useOurHalf, _useOpponentHalf;

    // True if we are running with a simulator.
    // This changes network communications.
    bool _simulation;

    // True if we are blue.
    // False if we are yellow.
    bool _blueTeam;

    // A logfile to read from.
    // When empty, don't read logs at all.
    std::string _readLogFile;

    // Locked when processing loop stuff is happening (not when blocked for
    // timing or I/O). This is public so the GUI thread can lock it to access
    // SystemState, etc.
    QMutex _loopMutex;

    /** global system state */
    Context _context;

    // Transformation from world space to team space.
    // This depends on which goal we're defending.
    //
    // _teamTrans is used for positions, not angles.
    // _teamAngle is used for angles.
    Geometry2d::TransformMatrix _worldToTeam;
    float _teamAngle;

    // Board ID of the robot to manually control or -1 if none
    int _manualID;
    // Use multiple joysticks at once
    bool _multipleManual;

    // Processing period in microseconds
    RJ::Seconds _framePeriod = RJ::Seconds(1) / 60;

    /// Measured framerate
    float _framerate;

    // This is used by the GUI to indicate status of the processing loop and
    // network
    QMutex _statusMutex;
    Status _status;

    // modules
    std::shared_ptr<VisionFilter> _vision;
    std::shared_ptr<Referee> _refereeModule;
    std::shared_ptr<Gameplay::GameplayModule> _gameplayModule;
    std::unique_ptr<Planning::MultiRobotPathPlanner> _pathPlanner;
    std::unique_ptr<VisionReceiver> _visionReceiver;
    std::unique_ptr<MotionControlNode> _motionControl;
    std::unique_ptr<RadioNode> _radio;
    std::unique_ptr<GrSimCommunicator> _grSimCom;

    std::vector<Node*> _nodes;

    // joystick control
    std::vector<Joystick*> _joysticks;

    // joystick damping
    bool _dampedRotation;
    bool _dampedTranslation;

    bool _kickOnBreakBeam;

    // If true, rotates robot commands from the joystick based on its
    // orientation on the field
    bool _useFieldOrientedManualDrive = false;

    VisionChannel _visionChannel;

    bool _initialized;

    bool _paused;
};
