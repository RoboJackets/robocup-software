#include "Processor.hpp"

#include <QMutexLocker>

#include <Geometry2d/Util.hpp>
#include <LogUtils.hpp>
#include <Robot.hpp>
#include <RobotConfig.hpp>
#include <gameplay/GameplayModule.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_protos/messages_robocup_ssl_detection.pb.h>

#include "DebugDrawer.hpp"
#include "radio/PacketConvert.hpp"
#include "radio/RadioNode.hpp"

REGISTER_CONFIGURABLE(Processor)

using namespace boost;
using namespace Geometry2d;
using namespace google::protobuf;

// TODO: Remove this and just use the one in Context.
Field_Dimensions* current_dimensions = &Field_Dimensions::Current_Dimensions;

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

Processor::Processor(bool sim, bool blue_team, const std::string& read_log_file)
    : _loopMutex(), _readLogFile(read_log_file) {
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

    _context.field_dimensions = *current_dimensions;

    _ros_executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    _referee_sub =
        std::make_unique<ros2_temp::RefereeSub>(&_context, _ros_executor.get());
    _gameplayModule = std::make_shared<Gameplay::GameplayModule>(&_context);
    _motionControl = std::make_unique<MotionControlNode>(&_context);
    _planner_node = std::make_unique<Planning::PlannerNode>(&_context);
    _radio = std::make_unique<RadioNode>(&_context, sim, blue_team);
    _grSimCom = std::make_unique<GrSimCommunicator>(&_context);
    _logger = std::make_unique<Logger>(&_context);

    // ROS2 temp nodes
    _config_client = std::make_unique<ros2_temp::SoccerConfigClient>(&_context);
    _raw_vision_packet_sub =
        std::make_unique<ros2_temp::RawVisionPacketSub>(&_context);
    _world_state_queue = std::make_unique<AsyncWorldStateMsgQueue>(
        "world_state_queue", vision_filter::topics::kWorldStatePub);

    // Joystick
    _sdl_joystick_node = std::make_unique<joystick::SDLJoystickNode>(&_context);
    _manual_control_node =
        std::make_unique<joystick::ManualControlNode>(&_context);

    if (!read_log_file.empty()) {
        _logger->read(read_log_file);
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

/**
 * program loop
 */
void Processor::run() {
    Status cur_status;

    bool first = true;
    // main loop
    while (_running) {
        RJ::Time start_time = RJ::now();
        auto delta_time = start_time - cur_status.lastLoopTime;
        _framerate = RJ::Seconds(1) / delta_time;
        cur_status.lastLoopTime = start_time;
        _context.state.time = start_time;

        // Don't run processor while we're paused or reading logs after the
        // first cycle (we need to run one because MainWindow waits on a single
        // cycle of processor to initialize).
        while (_initialized && _running &&
               (_context.game_settings.paused ||
                _context.logs.state == Logs::State::kReading)) {
            std::this_thread::sleep_for(RJ::Seconds(1.0 / 60.0));
        }

        ////////////////
        // Inputs
        // TODO(#1558): Backport spin_all and use it for our main executor.
        _ros_executor->spin_some();
        _sdl_joystick_node->run();
        _manual_control_node->run();

        // Updates context_->field_dimensions
        _config_client->run();

        // Updates context_->raw_vision_packets
        _raw_vision_packet_sub->run();

        if (_context.field_dimensions != *current_dimensions) {
            std::cout << "Updating field geometry based off of vision packet."
                      << std::endl;
            setFieldDimensions(_context.field_dimensions);
        }

        _radio->run();

        if (_radio) {
            cur_status.lastRadioRxTime = _radio->getLastRadioRxTime();
        }

        const WorldStateMsg::SharedPtr world_state_msg =
            _world_state_queue->Get();
        if (world_state_msg != nullptr) {
            _context.world_state =
                rj_convert::convert_from_ros(*world_state_msg);
            cur_status.lastVisionTime =
                rj_convert::convert_from_ros(world_state_msg->last_update_time);
        }

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

        // TODO(#1505): Run all modules in sequence using the vector. For now we
        // still have to update things manually.

        ////////////////
        // Outputs

        if (_context.game_state.halt()) {
            stopRobots();
        }
        updateIntentActive();
        _manual_control_node->run();

        // Store processing loop status
        _statusMutex.lock();
        _status = cur_status;
        _statusMutex.unlock();

        // Processor Initialization Completed
        _initialized = true;

        {
            loopMutex()->lock();
            // Log this entire frame
            _logger->run();
            loopMutex()->unlock();
        }

        ////////////////
        // Timing

        auto end_time = RJ::now();
        auto time_lapse = end_time - start_time;
        if (time_lapse < _framePeriod) {
            ::usleep(RJ::numMicroseconds(_framePeriod - time_lapse));
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

void Processor::setFieldDimensions(const Field_Dimensions& dims) {
    *current_dimensions = dims;
    _gameplayModule->calculateFieldObstacles();
    _gameplayModule->updateFieldDimensions();
}

bool Processor::isRadioOpen() const { return _radio->isOpen(); }
bool Processor::isInitialized() const { return _initialized; }
