#include "processor.hpp"

#include <QMutexLocker>
#include <spdlog/spdlog.h>

#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_geometry/util.hpp>
#include <rj_utils/logging.hpp>

#include "debug_drawer.hpp"
#include "gameplay/gameplay_module.hpp"
#include "radio/packet_convert.hpp"
#include "robot.hpp"
#include "robot_config.hpp"

REGISTER_CONFIGURABLE(Processor)

using namespace boost;
using namespace rj_geometry;
using namespace google::protobuf;

// TODO: Remove this and just use the one in Context.
FieldDimensions* current_dimensions = &FieldDimensions::current_dimensions;

// A temporary place to store RobotLocalConfig/RobotConfig variables as we
// create them. They are initialized in create_configuration, before the
// Processor class is initialized, so we need to temporarily store them
// somewhere.
std::vector<RobotLocalConfig> robot_status_init;
std::unique_ptr<RobotConfig> Processor::robot_config_init;

void Processor::create_configuration(Configuration* cfg) {
    // If robot_config_init is not null, then we've already done this.
    // That means we're doing FromRegisteredConfigurables() in python code,
    // and so we shouldn't reinitialize anything.
    if (robot_config_init) {
        return;
    }

    robot_config_init = std::make_unique<RobotConfig>(cfg, "Rev2015");

    for (size_t s = 0; s < kNumShells; ++s) {
        robot_status_init.emplace_back(cfg, QString("Robot Statuses/Robot %1").arg(s));
    }
}

Processor::Processor(bool sim, bool blue_team, const std::string& read_log_file)
    : loop_mutex_(), read_log_file_(read_log_file) {
    // Set the logger to ros2.
    rj_utils::set_spdlog_default_ros2("processor");

    running_ = true;
    framerate_ = 0;
    initialized_ = false;

    // Configuration-time variables.
    context_.robot_config = std::move(robot_config_init);
    for (int i = kNumShells - 1; i >= 0; i--) {
        // Set up fields in Context
        context_.local_configs[i] = std::move(robot_status_init.back());
        robot_status_init.pop_back();
    }

    context_.field_dimensions = *current_dimensions;

    ros_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    gameplay_module_ = std::make_shared<Gameplay::GameplayModule>(&context_);
    gr_sim_com_ = std::make_unique<GrSimCommunicator>(&context_);
    logger_ = std::make_unique<Logger>(&context_);

    // ROS2 temp nodes
    config_client_ = std::make_unique<ros2_temp::SoccerConfigClient>(&context_);
    raw_vision_packet_sub_ = std::make_unique<ros2_temp::RawVisionPacketSub>(&context_);
    referee_sub_ = std::make_unique<ros2_temp::RefereeSub>(&context_, ros_executor_.get());
    debug_draw_sub_ =
        std::make_unique<ros2_temp::DebugDrawInterface>(&context_, ros_executor_.get());
    autonomy_interface_ =
        std::make_unique<ros2_temp::AutonomyInterface>(&context_, ros_executor_.get());

    world_state_queue_ = std::make_unique<AsyncWorldStateMsgQueue>(
        "world_state_queue", vision_filter::topics::kWorldStatePub);

    // Joystick
    sdl_joystick_node_ = std::make_unique<joystick::SDLJoystickNode>(&context_);
    manual_control_node_ = std::make_unique<joystick::ManualControlNode>(&context_);

    if (!read_log_file.empty()) {
        logger_->read(read_log_file);
    }

    logger_->start();

    nodes_.push_back(gr_sim_com_.get());
    nodes_.push_back(logger_.get());
}

Processor::~Processor() {
    stop();

    // Put back configurables where we found them.
    // This is kind of a hack, but if we don't do that they get destructed
    // when Processor dies. That normally isn't a problem, but in unit tests,
    // we create and destroy multiple instances of Processor for each test.
    robot_config_init = std::move(context_.robot_config);

    for (size_t i = 0; i < kNumShells; i++) {
        robot_status_init.push_back(std::move(context_.local_configs[i]));
    }
}

void Processor::stop() {
    if (running_) {
        running_ = false;
    }
}

/**
 * program loop
 */
void Processor::run() {
    Status cur_status;

    bool first = true;
    // main loop
    while (running_) {
        RJ::Time start_time = RJ::now();
        auto delta_time = start_time - cur_status.last_loop_time;
        framerate_ = RJ::Seconds(1) / delta_time;
        cur_status.last_loop_time = start_time;
        context_.state.time = start_time;

        // Don't run processor while we're paused or reading logs after the
        // first cycle (we need to run one because MainWindow waits on a single
        // cycle of processor to initialize).
        while (initialized_ && running_ &&
               (context_.game_settings.paused || context_.logs.state == Logs::State::kReading)) {
            std::this_thread::sleep_for(RJ::Seconds(1.0 / 60.0));
        }

        ////////////////
        // Inputs
        // TODO(#1558): Backport spin_all and use it for our main executor.
        for (int i = 0; i < 10; i++) {
            ros_executor_->spin_some();
        }
        sdl_joystick_node_->run();
        manual_control_node_->run();

        // Updates context_->field_dimensions
        config_client_->run();

        // Updates context_->raw_vision_packets
        raw_vision_packet_sub_->run();

        if (context_.field_dimensions != *current_dimensions) {
            SPDLOG_INFO("Updating field geometry based off of vision packet.");
            set_field_dimensions(context_.field_dimensions);
        }

        const WorldStateMsg::SharedPtr world_state_msg = world_state_queue_->get();
        if (world_state_msg != nullptr) {
            context_.world_state = rj_convert::convert_from_ros(*world_state_msg);
            cur_status.last_vision_time =
                rj_convert::convert_from_ros(world_state_msg->last_update_time);
        }

        // Run high-level soccer logic
        gameplay_module_->run();

        update_intent_active();

        autonomy_interface_->run();

        // recalculates Field obstacles on every run through to account for
        // changing inset
        if (gameplay_module_->has_field_edge_inset_changed()) {
            gameplay_module_->calculate_field_obstacles();
        }

        gr_sim_com_->run();

        // TODO(#1505): Run all modules in sequence using the vector. For now we
        // still have to update things manually.

        ////////////////
        // Outputs

        if (context_.game_state.halt()) {
            stop_robots();
        }
        manual_control_node_->run();

        // Store processing loop status
        status_mutex_.lock();
        status_ = cur_status;
        status_mutex_.unlock();

        // Processor Initialization Completed
        initialized_ = true;

        debug_draw_sub_->run();

        {
            loop_mutex()->lock();
            // Log this entire frame
            logger_->run();
            loop_mutex()->unlock();
        }

        ////////////////
        // Timing

        auto end_time = RJ::now();
        auto time_lapse = end_time - start_time;
        if (time_lapse < frame_period_) {
            ::usleep(RJ::num_microseconds(frame_period_ - time_lapse));
        } else {
            //   printf("Processor took too long: %d us\n", last_frame_time);
        }
    }
}

void Processor::stop_robots() {
    for (OurRobot* r : context_.state.self) {
        RobotIntent& intent = context_.robot_intents[r->shell()];
        MotionSetpoint& setpoint = context_.motion_setpoints[r->shell()];

        intent = {};
        setpoint = {};
    }
}

void Processor::update_intent_active() {
    // Intent is active if it's being joystick controlled, or
    // if it's visible.
    for (OurRobot* r : context_.state.self) {
        RobotIntent& intent = context_.robot_intents[r->shell()];
        intent.is_active = r->is_joystick_controlled() || r->visible();
    }
}

void Processor::set_field_dimensions(const FieldDimensions& dims) {
    *current_dimensions = dims;
    gameplay_module_->calculate_field_obstacles();
    gameplay_module_->update_field_dimensions();
}

bool Processor::is_initialized() const { return initialized_; }
