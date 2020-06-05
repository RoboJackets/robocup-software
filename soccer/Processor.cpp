#include "Processor.hpp"

#include <protobuf/messages_robocup_ssl_detection.pb.h>

#include <Constants.hpp>
#include <Geometry2d/Util.hpp>
#include <LogUtils.hpp>
#include <QMutexLocker>
#include <Robot.hpp>
#include <RobotConfig.hpp>
#include <Utils.hpp>
#include <gameplay/GameplayModule.hpp>

#include "DebugDrawer.hpp"
#include "radio/PacketConvert.hpp"
#include "radio/RadioNode.hpp"
#include "vision/VisionFilter.hpp"

REGISTER_CONFIGURABLE(Processor)

using namespace boost;
using namespace Geometry2d;
using namespace google::protobuf;

// TODO: Remove this and just use the one in Context.
Field_Dimensions* currentDimensions = &Field_Dimensions::Current_Dimensions;

// A temporary place to store RobotLocalConfig/RobotConfig variables as we
// create them. They are initialized in createConfiguration, before the
// Processor class is initialized, so we need to temporarily store them
// somewhere.
std::vector<RobotLocalConfig> robot_status_init;
std::unique_ptr<RobotConfig> Processor::robot_config_init;

void Processor::createConfiguration(Configuration* cfg) {
    // If robot_config_init is not null, then we've already done this.
    // That means we're doing FromRegisteredConfigurables() in python code,
    // and so we shouldn't reinitialize anything.
    if (robot_config_init) {
        return;
    }

    robot_config_init = std::make_unique<RobotConfig>(cfg, "Rev2015");

    for (size_t s = 0; s < Num_Shells; ++s) {
        robot_status_init.emplace_back(
            cfg, QString("Robot Statuses/Robot %1").arg(s));
    }
}

Processor::Processor(bool sim, bool blueTeam, const std::string& readLogFile)
    : _loopMutex(), _readLogFile(readLogFile) {
    _running = true;
    _framerate = 0;
    _initialized = false;
    _radio = nullptr;

    // Configuration-time variables.
    _context.robot_config = std::move(robot_config_init);
    for (int i = Num_Shells - 1; i >= 0; i--) {
        // Set up fields in Context
        _context.local_configs[i] = std::move(robot_status_init.back());
        robot_status_init.pop_back();
    }

    _context.field_dimensions = *currentDimensions;

    _vision = std::make_shared<VisionFilter>();
    _refereeModule = std::make_shared<Referee>(&_context);
    _refereeModule->start();
    _gameplayModule = std::make_shared<Gameplay::GameplayModule>(
        &_context, _refereeModule.get());
    _motionControl = std::make_unique<MotionControlNode>(&_context);
    _planner_node = std::make_unique<Planning::PlannerNode>(&_context);
    _radio = std::make_unique<RadioNode>(&_context, sim, blueTeam);
//    _visionReceiver = std::make_unique<VisionReceiver>(
//        &_context, sim, sim ? SimVisionPort : SharedVisionPortSinglePrimary);
    _grSimCom = std::make_unique<GrSimCommunicator>(&_context);
    _logger = std::make_unique<Logger>(&_context);

    // Joystick
    _sdl_joystick_node = std::make_unique<joystick::SDLJoystickNode>(&_context);
    _manual_control_node =
        std::make_unique<joystick::ManualControlNode>(&_context);

    if (!readLogFile.empty()) {
        _logger->read(readLogFile);
    }

    _logger->start();

    _nodes.push_back(_motionControl.get());
    _nodes.push_back(_grSimCom.get());
    _nodes.push_back(_logger.get());
}

Processor::~Processor() {
    stop();

    // Put back configurables where we found them.
    // This is kind of a hack, but if we don't do that they get destructed
    // when Processor dies. That normally isn't a problem, but in unit tests,
    // we create and destroy multiple instances of Processor for each test.
    robot_config_init = std::move(_context.robot_config);

    for (size_t i = 0; i < Num_Shells; i++) {
        robot_status_init.push_back(std::move(_context.local_configs[i]));
    }
}

void Processor::stop() {
    if (_running) {
        _running = false;
    }
}

void Processor::runModels() {
    std::vector<CameraFrame> frames;

    for (auto& packet : _context.vision_packets) {
        const SSL_DetectionFrame* frame = packet->wrapper.mutable_detection();
        std::vector<CameraBall> ballObservations;
        std::vector<CameraRobot> yellowObservations;
        std::vector<CameraRobot> blueObservations;

        RJ::Time time =
            RJ::Time(std::chrono::duration_cast<std::chrono::microseconds>(
                RJ::Seconds(frame->t_capture())));

        // Add ball observations
        ballObservations.reserve(frame->balls().size());
        for (const SSL_DetectionBall& ball : frame->balls()) {
            ballObservations.emplace_back(
                time, _worldToTeam * Point(ball.x() / 1000, ball.y() / 1000));
        }

        // Collect camera data from all robots
        yellowObservations.reserve(frame->robots_yellow().size());
        for (const SSL_DetectionRobot& robot : frame->robots_yellow()) {
            yellowObservations.emplace_back(
                time,
                Pose(Point(_worldToTeam *
                           Point(robot.x() / 1000, robot.y() / 1000)),
                     fixAngleRadians(robot.orientation() + _teamAngle)),
                robot.robot_id());
        }

        // Collect camera data from all robots
        blueObservations.reserve(frame->robots_blue().size());
        for (const SSL_DetectionRobot& robot : frame->robots_blue()) {
            blueObservations.emplace_back(
                time,
                Pose(Point(_worldToTeam *
                           Point(robot.x() / 1000, robot.y() / 1000)),
                     fixAngleRadians(robot.orientation() + _teamAngle)),
                robot.robot_id());
        }

        frames.emplace_back(time, frame->camera_id(), ballObservations,
                            yellowObservations, blueObservations);
    }

    _vision->addFrames(frames);

    // Fill the list of our robots/balls based on whether we are the blue team
    // or not
    _vision->fillBallState(_context.state);
    _vision->fillRobotState(_context.state, _context.game_state.blueTeam);
}

/**
 * program loop
 */
void Processor::run() {
    Status curStatus;

    bool first = true;
    // main loop
    while (_running) {
        RJ::Time startTime = RJ::now();
        auto deltaTime = startTime - curStatus.lastLoopTime;
        _framerate = RJ::Seconds(1) / deltaTime;
        curStatus.lastLoopTime = startTime;
        _context.state.time = startTime;

        // Don't run processor while we're paused or reading logs after the
        // first cycle (we need to run one because MainWindow waits on a single
        // cycle of processor to initialize).
        while (_initialized && _running &&
               (_context.game_settings.paused ||
                _context.logs.state == Logs::State::kReading)) {
            std::this_thread::sleep_for(RJ::Seconds(1.0 / 60.0));
        }

        loopMutex()->lock();

        ////////////////
        // Inputs
        _sdl_joystick_node->run();
        _manual_control_node->run();

        updateOrientation();

        // TODO(Kyle): Don't do this here.
        // Because not everything is on modules yet, but we still need things to
        // run in order, we can't just do everything via the for loop (yet).
//        _visionReceiver->run();

        if (_context.field_dimensions != *currentDimensions) {
            std::cout << "Updating field geometry based off of vision packet."
                      << std::endl;
            setFieldDimensions(_context.field_dimensions);
        }

//        curStatus.lastVisionTime = _visionReceiver->getLastVisionTime();

        _radio->run();

        if (_radio) {
            curStatus.lastRadioRxTime = _radio->getLastRadioRxTime();
        }

        runModels();

        _context.vision_packets.clear();

        // Log referee data
        _refereeModule->run();

        // Run high-level soccer logic
        _gameplayModule->run();

        // recalculates Field obstacles on every run through to account for
        // changing inset
        if (_gameplayModule->hasFieldEdgeInsetChanged()) {
            _gameplayModule->calculateFieldObstacles();
        }

        // In: Global Obstacles
        // Out: context_->trajectories
        _planner_node->run();

        // In: context_->trajectories
        // Out: context_->motion_setpoints
        _motionControl->run();

        _grSimCom->run();

        // Run all nodes in sequence
        // TODO(Kyle): This is dead code for now. Once everything is ported over
        // to modules we can delete the if (false), but for now we still have to
        // update things manually.

        ////////////////
        // Outputs

        if (_context.game_state.halt()) {
            stopRobots();
        }
        updateIntentActive();
        _manual_control_node->run();

        // Store processing loop status
        _statusMutex.lock();
        _status = curStatus;
        _statusMutex.unlock();

        // Processor Initialization Completed
        _initialized = true;

        // Log this entire frame
        _logger->run();

        ////////////////
        // Timing

        loopMutex()->unlock();

        auto endTime = RJ::now();
        auto timeLapse = endTime - startTime;
        if (timeLapse < _framePeriod) {
            ::usleep(RJ::numMicroseconds(_framePeriod - timeLapse));
        } else {
            //   printf("Processor took too long: %d us\n", lastFrameTime);
        }
    }
}

void Processor::stopRobots() {
    for (OurRobot* r : _context.state.self) {
        RobotIntent& intent = _context.robot_intents[r->shell()];
        MotionSetpoint& setpoint = _context.motion_setpoints[r->shell()];

        setpoint.clear();
        intent.dvelocity = 0;
        intent.kcstrength = 0;
        intent.shoot_mode = RobotIntent::ShootMode::KICK;
        intent.trigger_mode = RobotIntent::TriggerMode::STAND_DOWN;
        intent.song = RobotIntent::Song::STOP;
    }
}

void Processor::updateIntentActive() {
    // Intent is active if it's being joystick controlled, or
    // if it's visible.
    for (OurRobot* r : _context.state.self) {
        RobotIntent& intent = _context.robot_intents[r->shell()];
        intent.is_active = r->isJoystickControlled() || r->visible();
    }
}

void Processor::recalculateWorldToTeamTransform() {
    _worldToTeam = Geometry2d::TransformMatrix::translate(
        0, Field_Dimensions::Current_Dimensions.Length() / 2.0f);
    _worldToTeam *= Geometry2d::TransformMatrix::rotate(_teamAngle);
}

void Processor::setFieldDimensions(const Field_Dimensions& dims) {
    *currentDimensions = dims;
    recalculateWorldToTeamTransform();
    _gameplayModule->calculateFieldObstacles();
    _gameplayModule->updateFieldDimensions();
}

bool Processor::isRadioOpen() const { return _radio->isOpen(); }
bool Processor::isInitialized() const { return _initialized; }

void Processor::updateOrientation() {
    if (_context.game_settings.defendPlusX) {
        _teamAngle = -M_PI_2;
    } else {
        _teamAngle = M_PI_2;
    }

    recalculateWorldToTeamTransform();
}
