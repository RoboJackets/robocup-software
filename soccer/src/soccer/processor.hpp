
// FIXME - Move a lot of stuff like blueTeam and worldToTeam to a globally
// accessible place

#pragma once

#include <rj_protos/LogFrame.pb.h>
#include <rj_topic_utils/async_message_queue.h>
#include <ros2_temp/raw_vision_packet_sub.h>
#include <ros2_temp/soccer_config_client.h>

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Pose.hpp>
#include <Geometry2d/TransformMatrix.hpp>
#include <logger.hpp>
#include <system_state.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <referee/external_referee.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <ros2_temp/referee_sub.hpp>
#include <vector>

#include "context.hpp"
#include "gr_sim_communicator.hpp"
#include "node.hpp"
#include "joystick/manual_control_node.hpp"
#include "joystick/sdl_joystick_node.hpp"
#include "motion/motion_control_node.hpp"
#include "planning/planner_node.hpp"
#include "radio/radio.hpp"
#include "radio/radio_node.hpp"
#include "rc-fshare/rtp.hpp"

class Configuration;
class RobotLocalConfig;
class Joystick;
struct JoystickControlValues;
class Radio;

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

        RJ::Time last_loop_time;
        RJ::Time last_vision_time;
        RJ::Time last_referee_time;
        RJ::Time last_radio_rx_time;
    };

    static void create_configuration(Configuration* cfg);

    Processor(bool sim, bool blue_team, const std::string& read_log_file = "");
    virtual ~Processor();

    void stop();

    std::shared_ptr<Gameplay::GameplayModule> gameplay_module() const {
        return gameplay_module_;
    }

    SystemState* state() { return &context_.state; }

    Status status() {
        std::lock_guard lock(status_mutex_);
        return status_;
    }

    float framerate() { return framerate_; }

    bool open_log(const QString& filename) {
        logger_->write(filename.toStdString());
        return true;
    }

    void close_log() { logger_->close(); }

    std::lock_guard<std::mutex> lock_loop_mutex() {
        return std::lock_guard(loop_mutex_);
    }

    std::mutex* loop_mutex() { return &loop_mutex_; }

    Radio* radio() { return radio_->get_radio(); }

    /**
     * Stops all robots by clearing their intents and setpoints
     */
    void stop_robots();

    void set_field_dimensions(const FieldDimensions& dims);

    bool is_radio_open() const;

    bool is_initialized() const;

    Context* context() { return &context_; }

    void run();

private:
    // Configuration for the robot.
    // TODO(Kyle): Add back in configuration values for different years.
    static std::unique_ptr<RobotConfig> robot_config_init;

    // per-robot status configs
    static std::vector<RobotLocalConfig*> robot_statuses;

    /**
     * Updates the intent.active for each robot.
     *
     * The intent is active if it's being joystick controlled or
     * if it's visible
     */
    void update_intent_active();

    /** Used to start and stop the thread **/
    volatile bool running_;

    // A logfile to read from.
    // When empty, don't read logs at all.
    std::string read_log_file_;

    // Locked when processing loop stuff is happening (not when blocked for
    // timing or I/O). This is public so the GUI thread can lock it to access
    // SystemState, etc.
    std::mutex loop_mutex_;

    /** global system state */
    Context context_;

    // Processing period in microseconds
    RJ::Seconds frame_period_ = RJ::Seconds(1) / 60;

    /// Measured framerate
    float framerate_;

    // This is used by the GUI to indicate status of the processing loop and
    // network
    std::mutex status_mutex_;
    Status status_;

    // modules
    std::shared_ptr<Gameplay::GameplayModule> gameplay_module_;
    std::unique_ptr<MotionControlNode> motion_control_;
    std::unique_ptr<Planning::PlannerNode> planner_node_;
    std::unique_ptr<RadioNode> radio_;
    std::unique_ptr<GrSimCommunicator> gr_sim_com_;
    std::unique_ptr<joystick::SDLJoystickNode> sdl_joystick_node_;
    std::unique_ptr<joystick::ManualControlNode> manual_control_node_;
    std::unique_ptr<Logger> logger_;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor_;

    // ROS2 temporary modules
    using WorldStateMsg = rj_msgs::msg::WorldState;
    using AsyncWorldStateMsgQueue = rj_topic_utils::AsyncMessageQueue<
        WorldStateMsg, rj_topic_utils::MessagePolicy::kQueue, 1>;

    AsyncWorldStateMsgQueue::UniquePtr world_state_queue_;

    std::unique_ptr<ros2_temp::SoccerConfigClient> config_client_;
    std::unique_ptr<ros2_temp::RawVisionPacketSub> raw_vision_packet_sub_;
    std::unique_ptr<ros2_temp::RefereeSub> referee_sub_;

    std::vector<Node*> nodes_;

    bool initialized_;
};
