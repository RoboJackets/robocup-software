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
#include <joystick/GamepadController.hpp>
#include <joystick/Joystick.hpp>
#include <planning/IndependentMultiRobotPathPlanner.hpp>
#include <rc-fshare/git_version.hpp>

#include "DebugDrawer.hpp"
#include "radio/PacketConvert.hpp"
#include "radio/RadioNode.hpp"
#include "vision/VisionFilter.hpp"

REGISTER_CONFIGURABLE(Processor)

using namespace boost;
using namespace Geometry2d;
using namespace google::protobuf;

static const auto Command_Latency = 0ms;

// TODO: Remove this and just use the one in Context.
Field_Dimensions* currentDimensions = &Field_Dimensions::Current_Dimensions;

// A temporary place to store RobotStatus/RobotConfig variables as we create
// them. They are initialized in createConfiguration, before the Processor class
// is initialized, so we need to temporarily store them somewhere.
std::vector<RobotStatus> robot_status_init;
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

    setupJoysticks();

    // Configuration-time variables.
    _context.robot_config = std::move(robot_config_init);
    for (int i = Num_Shells - 1; i >= 0; i--) {
        // Set up fields in Context
        _context.robot_status[i] = std::move(robot_status_init.back());
        robot_status_init.pop_back();
    }

    _context.field_dimensions = *currentDimensions;

    _vision = std::make_shared<VisionFilter>();
    _refereeModule = std::make_shared<Referee>(&_context);
    _refereeModule->start();
    _gameplayModule = std::make_shared<Gameplay::GameplayModule>(
        &_context, _refereeModule.get());
    _pathPlanner = std::unique_ptr<Planning::MultiRobotPathPlanner>(
        new Planning::IndependentMultiRobotPathPlanner());
    _motionControl = std::make_unique<MotionControlNode>(&_context);
    _radio = std::make_unique<RadioNode>(&_context, sim, blueTeam);
    _visionReceiver = std::make_unique<VisionReceiver>(
        &_context, sim, sim ? SimVisionPort : SharedVisionPortSinglePrimary);
    _grSimCom = std::make_unique<GrSimCommunicator>(&_context);

    // Joystick
    _sdl_joystick_node = std::make_unique<joystick::SDLJoystickNode>();
    _manual_control_node =
        std::make_unique<joystick::ManualControlNode>(&_context);

    // Register callbacks
    _sdl_joystick_node->addCallbacks(_manual_control_node->getCallback(),
                                     _manual_control_node->getOnConnect(),
                                     _manual_control_node->getOnDisconnect());

    if (!readLogFile.empty()) {
        _logger.readFrames(readLogFile.c_str());
        firstLogTime = _logger.startTime();
    }

    _nodes.push_back(_visionReceiver.get());
    _nodes.push_back(_motionControl.get());
    _nodes.push_back(_grSimCom.get());
}

Processor::~Processor() {
    stop();

    for (Joystick* joy : _joysticks) {
        delete joy;
    }

    // Put back configurables where we found them.
    // This is kind of a hack, but if we don't do that they get destructed
    // when Processor dies. That normally isn't a problem, but in unit tests,
    // we create and destroy multiple instances of Processor for each test.
    robot_config_init = std::move(_context.robot_config);

    for (size_t i = 0; i < Num_Shells; i++) {
        robot_status_init.push_back(std::move(_context.robot_status[i]));
    }
}

void Processor::stop() {
    if (_running) {
        _running = false;
    }
}

void Processor::setupJoysticks() {
    _joysticks.clear();

    GamepadController::controllersInUse.clear();
    GamepadController::joystickRemoved = -1;

    for (int i = 0; i < Robots_Per_Team; i++) {
        _joysticks.push_back(new GamepadController());
    }

    //_joysticks.push_back(new SpaceNavJoystick()); //Add this back when
    // isValid() is working properly
}

bool Processor::joystickValid() const {
    for (Joystick* joy : _joysticks) {
        if (joy->valid()) {
            return true;
        }
    }
    return false;
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

        if (!firstLogTime) {
            firstLogTime = startTime;
        }

        ////////////////
        // Reset

        // Make a new log frame
        _context.state.logFrame = std::make_shared<Packet::LogFrame>();
        _context.state.logFrame->set_timestamp(RJ::timestamp());
        _context.state.logFrame->set_command_time(
            RJ::timestamp(startTime + Command_Latency));
        _context.state.logFrame->set_use_our_half(
            _context.game_settings.use_our_half);
        _context.state.logFrame->set_use_opponent_half(
            _context.game_settings.use_their_half);
        _context.state.logFrame->set_manual_id(
            _context.game_settings.joystick_config.manualID);
        _context.state.logFrame->set_blue_team(_context.game_state.blueTeam);
        _context.state.logFrame->set_defend_plus_x(
            _context.game_settings.defendPlusX);
        _context.debug_drawer.setLogFrame(_context.state.logFrame.get());

        if (first) {
            first = false;

            Packet::LogConfig* logConfig =
                _context.state.logFrame->mutable_log_config();
            logConfig->set_generator("soccer");
            logConfig->set_git_version_hash(git_version_hash);
            logConfig->set_git_version_dirty(git_version_dirty);
            logConfig->set_simulation(_context.game_settings.simulation);
        }

        ////////////////
        // Inputs
        _sdl_joystick_node->run();

        updateOrientation();

        // TODO(Kyle): Don't do this here.
        // Because not everything is on modules yet, but we still need things to
        // run in order, we can't just do everything via the for loop (yet).
        _visionReceiver->run();

        if (_context.field_dimensions != *currentDimensions) {
            std::cout << "Updating field geometry based off of vision packet."
                      << std::endl;
            setFieldDimensions(_context.field_dimensions);
        }

        curStatus.lastVisionTime = _visionReceiver->getLastVisionTime();

        _radio->run();

        if (_radio) {
            curStatus.lastRadioRxTime = _radio->getLastRadioRxTime();
        }

        for (Joystick* joystick : _joysticks) {
            joystick->update();
        }
        GamepadController::joystickRemoved = -1;

        runModels();

        _context.vision_packets.clear();

        // Log referee data
        _refereeModule->run();
        std::vector<RefereePacket> refereePackets;
        _refereeModule->getPackets(refereePackets);
        for (const RefereePacket& packet : refereePackets) {
            SSL_Referee* log = _context.state.logFrame->add_raw_refbox();
            log->CopyFrom(packet.wrapper);
            curStatus.lastRefereeTime =
                std::max(curStatus.lastRefereeTime, packet.receivedTime);
        }

        std::string yellowname;
        std::string bluename;

        if (_context.game_state.blueTeam) {
            bluename = _context.game_state.OurInfo.name;
            yellowname = _context.game_state.TheirInfo.name;
        } else {
            yellowname = _context.game_state.OurInfo.name;
            bluename = _context.game_state.TheirInfo.name;
        }

        _context.state.logFrame->set_team_name_blue(bluename);
        _context.state.logFrame->set_team_name_yellow(yellowname);

        // Run high-level soccer logic
        _gameplayModule->run();

        // recalculates Field obstacles on every run through to account for
        // changing inset
        if (_gameplayModule->hasFieldEdgeInsetChanged()) {
            _gameplayModule->calculateFieldObstacles();
        }
        /// Collect global obstacles
        Geometry2d::ShapeSet globalObstacles =
            _gameplayModule->globalObstacles();
        Geometry2d::ShapeSet globalObstaclesWithGoalZones = globalObstacles;
        Geometry2d::ShapeSet goalZoneObstacles =
            _gameplayModule->goalZoneObstacles();
        globalObstaclesWithGoalZones.add(goalZoneObstacles);

        // Build a plan request for each robot.
        std::map<int, Planning::PlanRequest> requests;
        for (OurRobot* r : _context.state.self) {
            if (r != nullptr && r->visible()) {
                if (_context.game_state.state == GameState::Halt) {
                    r->setPath(nullptr);
                    continue;
                }

                // Visualize local obstacles
                for (auto& shape : r->localObstacles().shapes()) {
                    _context.debug_drawer.drawShape(shape, Qt::black,
                                                    "LocalObstacles");
                }

                auto& globalObstaclesForBot =
                    (r->shell() == _context.game_state.getGoalieId() ||
                     r->isPenaltyKicker || r->isBallPlacer)
                        ? globalObstacles
                        : globalObstaclesWithGoalZones;

                // create and visualize obstacles
                Geometry2d::ShapeSet staticObstacles =
                    r->collectStaticObstacles(
                        globalObstaclesForBot,
                        !(r->shell() == _context.game_state.getGoalieId() ||
                          r->isPenaltyKicker || r->isBallPlacer));

                std::vector<Planning::DynamicObstacle> dynamicObstacles =
                    r->collectDynamicObstacles();

                requests.emplace(
                    r->shell(),
                    Planning::PlanRequest(
                        &_context, Planning::MotionInstant(r->pos(), r->vel()),
                        r->motionCommand()->clone(), r->robotConstraints(),
                        std::move(r->angleFunctionPath().path),
                        std::move(staticObstacles), std::move(dynamicObstacles),
                        r->shell(), r->getPlanningPriority()));
            }
        }

        // Run path planner and set the path for each robot that was planned for
        auto pathsById = _pathPlanner->run(std::move(requests));
        for (auto& entry : pathsById) {
            OurRobot* r = _context.state.self[entry.first];
            auto& path = entry.second;
            path->draw(&_context.debug_drawer, Qt::magenta, "Planning");
            path->drawDebugText(&_context.debug_drawer);
            r->setPath(std::move(path));

            r->angleFunctionPath().angleFunction =
                angleFunctionForCommandType(r->rotationCommand());
        }

        // Visualize obstacles
        for (auto& shape : globalObstacles.shapes()) {
            _context.debug_drawer.drawShape(shape, Qt::black,
                                            "Global Obstacles");
        }

        // TODO(Kyle, Collin): This is a horrible hack to get around the fact
        // that joystick code only (sort of) supports one joystick at a time.
        // Figure out which robots are manual controlled.
        for (OurRobot* robot : _context.state.self) {
            robot->setJoystickControlled(
                robot->shell() ==
                _context.game_settings.joystick_config.manualID);
        }

        _motionControl->run();
        _grSimCom->run();
        // Run all nodes in sequence
        // TODO(Kyle): This is dead code for now. Once everything is ported over
        // to modules we can delete the if (false), but for now we still have to
        // update things manually.

        ////////////////
        // Store logging information

        // Debug layers
        const QStringList& layers = _context.debug_drawer.debugLayers();
        for (const QString& str : layers) {
            _context.state.logFrame->add_debug_layers(str.toStdString());
        }

        // Add our robots data to the LogFrame
        for (OurRobot* r : _context.state.self) {
            if (r->visible()) {
                r->addStatusText();

                Packet::LogFrame::Robot* log =
                    _context.state.logFrame->add_self();
                *log->mutable_pos() = r->pos();
                *log->mutable_world_vel() = r->vel();
                *log->mutable_body_vel() =
                    r->vel().rotated(M_PI_2 - r->angle());
                //*log->mutable_cmd_body_vel() = r->
                // *log->mutable_cmd_vel() = r->cmd_vel;
                // log->set_cmd_w(r->cmd_w);
                log->set_shell(static_cast<float>(r->shell()));
                log->set_angle(static_cast<float>(r->angle()));
                auto radioRx = r->radioRx();
                if (radioRx.has_kicker_voltage()) {
                    log->set_kicker_voltage(radioRx.kicker_voltage());
                }

                if (radioRx.has_kicker_status()) {
                    log->set_charged((radioRx.kicker_status() & 0x01) != 0u);
                    log->set_kicker_works((radioRx.kicker_status() & 0x90) ==
                                          0u);
                }

                if (radioRx.has_ball_sense_status()) {
                    log->set_ball_sense_status(radioRx.ball_sense_status());
                }

                if (radioRx.has_battery()) {
                    log->set_battery_voltage(radioRx.battery());
                }

                log->mutable_motor_status()->Clear();
                log->mutable_motor_status()->MergeFrom(radioRx.motor_status());

                if (radioRx.has_quaternion()) {
                    log->mutable_quaternion()->Clear();
                    log->mutable_quaternion()->MergeFrom(radioRx.quaternion());
                } else {
                    log->clear_quaternion();
                }

                for (const Packet::DebugText& t : r->robotText) {
                    log->add_text()->CopyFrom(t);
                }
            }
        }

        // Opponent robots
        for (OpponentRobot* r : _context.state.opp) {
            if (r->visible()) {
                Packet::LogFrame::Robot* log =
                    _context.state.logFrame->add_opp();
                *log->mutable_pos() = r->pos();
                log->set_shell(r->shell());
                log->set_angle(r->angle());
                *log->mutable_world_vel() = r->vel();
                *log->mutable_body_vel() =
                    r->vel().rotated(2 * M_PI - r->angle());
            }
        }

        // Ball
        if (_context.state.ball.valid) {
            Packet::LogFrame::Ball* log =
                _context.state.logFrame->mutable_ball();
            *log->mutable_pos() = _context.state.ball.pos;
            *log->mutable_vel() = _context.state.ball.vel;
        }

        ////////////////
        // Outputs

        // Send motion commands to the robots
        sendRadioData();

        // Write to the log unless we are viewing logs or main window is paused
        if (_readLogFile.empty() && !_context.game_settings.paused) {
            _logger.addFrame(_context.state.logFrame);
        }

        // Store processing loop status
        _statusMutex.lock();
        _status = curStatus;
        _statusMutex.unlock();

        // Processor Initialization Completed
        _initialized = true;

        ////////////////
        // Timing

        auto endTime = RJ::now();
        auto timeLapse = endTime - startTime;
        if (timeLapse < _framePeriod) {
            ::usleep(RJ::numMicroseconds(_framePeriod - timeLapse));
        } else {
            //   printf("Processor took too long: %d us\n", lastFrameTime);
        }
    }
}

void Processor::sendRadioData() {
    // Halt overrides normal motion control, but not joystick
    if (_context.game_state.halt()) {
        // Force all motor speeds to zero
        for (OurRobot* r : _context.state.self) {
            RobotIntent& intent = _context.robot_intents[r->shell()];
            MotionSetpoint& setpoint = _context.motion_setpoints[r->shell()];
            setpoint.xvelocity = 0;
            setpoint.yvelocity = 0;
            setpoint.avelocity = 0;
            intent.dvelocity = 0;
            intent.kcstrength = 0;
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::STAND_DOWN;
            intent.song = RobotIntent::Song::STOP;
        }
    }

    // TODO(Kyle): Reimplement multiple manual code
    // Add RadioTx commands for visible robots and apply joystick input
    for (OurRobot* r : _context.state.self) {
        RobotIntent& intent = _context.robot_intents[r->shell()];
        if (_context.game_settings.joystick_config.manualID == r->shell()) {
            intent.is_active = true;
            auto controlValues = getJoystickControlValues();
            if (!controlValues.empty()) {
                applyJoystickControls(controlValues[0], r);
            }
        } else if (r->visible()) {
            intent.is_active = true;
        } else {
            intent.is_active = false;
        }
    }
}

void Processor::applyJoystickControls(const JoystickControlValues& controlVals,
                                      OurRobot* robot) {
    Geometry2d::Point translation(controlVals.translation);

    // use world coordinates if we can see the robot
    // otherwise default to body coordinates
    if (robot != nullptr && robot->visible() &&
        _context.game_settings.joystick_config.useFieldOrientedDrive) {
        translation.rotate(-M_PI / 2 - robot->angle());
    }
    RobotIntent& intent = _context.robot_intents[robot->shell()];
    MotionSetpoint& setpoint = _context.motion_setpoints[robot->shell()];
    // translation
    setpoint.xvelocity = translation.x();
    setpoint.yvelocity = translation.y();

    // rotation
    setpoint.avelocity = controlVals.rotation;

    // kick/chip
    bool kick = controlVals.kick || controlVals.chip;
    intent.trigger_mode =
        (kick ? (_context.game_settings.joystick_config.useKickOnBreakBeam
                     ? RobotIntent::TriggerMode::ON_BREAK_BEAM
                     : RobotIntent::TriggerMode::IMMEDIATE)
              : RobotIntent::TriggerMode::STAND_DOWN);
    intent.kcstrength = static_cast<int>(controlVals.kickPower);
    intent.shoot_mode = (controlVals.kick ? RobotIntent::ShootMode::KICK
                                          : RobotIntent::ShootMode::CHIP);

    // dribbler
    intent.dvelocity =
        static_cast<float>(controlVals.dribble ? controlVals.dribblerPower : 0);
}

JoystickControlValues Processor::getJoystickControlValue(Joystick& joy) {
    JoystickControlValues vals = joy.getJoystickControlValues();
    if (joy.valid()) {
        // keep it in range
        vals.translation.clamp(sqrt(2.0));
        if (vals.rotation > 1) {
            vals.rotation = 1;
        }
        if (vals.rotation < -1) {
            vals.rotation = -1;
        }

        // Gets values from the configured joystick control
        // values,respecting damped
        // state
        if (_context.game_settings.joystick_config.dampedTranslation) {
            vals.translation *=
                Joystick::JoystickTranslationMaxDampedSpeed->value();
        } else {
            vals.translation *= Joystick::JoystickTranslationMaxSpeed->value();
        }
        if (_context.game_settings.joystick_config.dampedRotation) {
            vals.rotation *= Joystick::JoystickRotationMaxDampedSpeed->value();
        } else {
            vals.rotation *= Joystick::JoystickRotationMaxSpeed->value();
        }

        // scale up kicker and dribbler speeds
        vals.dribblerPower *= Max_Dribble;
        vals.kickPower *= Max_Kick;
    }
    return vals;
}

std::vector<JoystickControlValues> Processor::getJoystickControlValues() {
    std::vector<JoystickControlValues> vals;
    for (Joystick* joy : _joysticks) {
        if (joy->valid()) {
            vals.push_back(getJoystickControlValue(*joy));
        }
    }
    return vals;
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
