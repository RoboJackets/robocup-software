#pragma once

#include <set>

#include <rj_constants/constants.hpp>
#include <rj_protos/referee.pb.h>

#include "rj_common/control/motion_setpoint.hpp"
#include "rj_common/debug_drawer.hpp"
#include "rj_common/game_settings.hpp"
#include "rj_common/game_state.hpp"
#include "rj_common/planning/robot_constraints.hpp"
#include "rj_common/planning/trajectory.hpp"
#include "rj_common/radio/robot_status.hpp"
#include "rj_common/robot_intent.hpp"
#include "rj_common/team_info.hpp"
#include "rj_common/world_state.hpp"

// Keep the past thirty minutes of logs by default.
constexpr size_t kMaxLogFrames = 60 * 60 * 30;

struct Logs {
    /**
     * \brief A list of all log frames. This will contain at most
     * `kMaxLogFrames` frames.
     *
     * This should not be accessed from the Processor thread, other
     * than to add frames. The container may be accessed by the MainWindow
     * thread while the Context mutex is locked, and frames may be retained
     * and used even while the mutex is not locked (provided a shared_ptr
     * is kept).
     */
    std::deque<std::shared_ptr<Packet::LogFrame>> frames;

    enum class State { kNoFile, kWriting, kReading };

    /**
     * \brief The name of the log file, if it exists.
     */
    std::optional<std::string> filename;

    /**
     * \brief Whether we are recording (or viewing) logs.
     */
    State state = State::kNoFile;

    /**
     * \brief The start time of the entire system.
     */
    RJ::Time start_time;

    /**
     * \brief The log file size in bytes.
     */
    size_t size_bytes = 0;

    /**
     * The count of frames that existed before the given history
     * but were dropped.
     */
    size_t dropped_frames = 0;
};

struct Context {
    Context() : debug_drawer(this) {}

    // Delete copy, copy-assign, move, and move-assign because
    // many places are expected to hold Context* pointers.
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    Context(Context&&) = delete;
    Context& operator=(Context&&) = delete;

    // Gameplay -> Planning, Radio
    std::array<RobotIntent, kNumShells> robot_intents;
    // Motion control -> Radio
    std::array<MotionSetpoint, kNumShells> motion_setpoints;
    // Planning -> Motion control
    std::array<planning::Trajectory, kNumShells> trajectories;
    // Radio -> Gameplay
    std::array<RobotStatus, kNumShells> robot_status;
    // Coach -> Positions
    // TODO(sid-parikh) Delete Robot_Positions UI stuff
    std::array<uint32_t, kNumShells> robot_positions;
    // MainWindow -> Manual control
    std::array<bool, kNumShells> is_joystick_controlled{};
    /** \brief Whether at least one joystick is connected */
    bool joystick_valid = false;

    rj_geometry::ShapeSet global_obstacles;
    rj_geometry::ShapeSet def_area_obstacles;

    PlayState play_state = PlayState::halt();
    MatchState match_state;

    TeamInfo our_info;
    TeamInfo their_info;
    bool blue_team = true;
    DebugDrawer debug_drawer;

    std::vector<Referee> referee_packets;
    std::vector<SSL_WrapperPacket> raw_vision_packets;

    WorldState world_state;

    FieldDimensions field_dimensions;

    GameSettings game_settings;

    Logs logs;
    std::string behavior_tree;
};
