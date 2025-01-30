#include "processor.hpp"

#include <QMutexLocker>
#include <spdlog/spdlog.h>

#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_geometry/util.hpp>
#include <rj_utils/logging.hpp>

#include "debug_drawer.hpp"
#include "radio/packet_convert.hpp"

using namespace boost;
using namespace rj_geometry;
using namespace google::protobuf;

// TODO: Remove this and just use the one in Context.
FieldDimensions* current_dimensions = &FieldDimensions::current_dimensions;

Processor::Processor(bool sim, bool blue_team, const std::string& read_log_file)
    : read_log_file_(read_log_file), loop_mutex_() {
    // Set the logger to ros2.
    rj_utils::set_spdlog_default_ros2("processor");

    running_ = true;
    framerate_ = 0;
    initialized_ = false;

    context_.field_dimensions = *current_dimensions;

    ros_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

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
        "world_state_queue", vision_filter::topics::kWorldStateTopic);

    if (!read_log_file.empty()) {
        logger_->read(read_log_file);
    }

    logger_->start();

    nodes_.push_back(logger_.get());
}

Processor::~Processor() { stop(); }

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
        for (int i = 0; rclcpp::ok() && i < 20; i++) {
            ros_executor_->spin_some();
        }
        // spin_all doesn't exist yet
        // ros_executor_->spin_all();

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

        autonomy_interface_->run();

        // TODO(#1505): Run all modules in sequence using the vector. For now we
        // still have to update things manually.

        ////////////////
        // Outputs

        if (context_.play_state.is_halt()) {
            stop_robots();
        }

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

void Processor::stop_robots() {}

void Processor::set_field_dimensions(const FieldDimensions& dims) { *current_dimensions = dims; }

bool Processor::is_initialized() const { return initialized_; }
