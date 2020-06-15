#pragma once

#include <rj_protos/LogFrame.pb.h>
#include <rj_protos/referee.pb.h>

#include <boost/asio.hpp>
#include <boost/config.hpp>
#include <cstdint>
#include <mutex>
#include <rj_common/RefereeEnums.hpp>
#include <rj_common/Utils.hpp>
#include <thread>
#include <vector>

#include "Context.hpp"
#include "GameState.hpp"
#include "Node.hpp"
#include "SystemState.hpp"
#include "TeamInfo.hpp"

/**
 * @brief A packet we received over the network from ssl-refbox
 *
 * @details Contains the protobuf packet from the refbox and a timestamp of when
 * we received it.
 */
struct RefereePacket {
    /// Local time when the packet was received
    RJ::Time receivedTime;

    /// protobuf message from the vision system
    SSL_Referee wrapper;
};

/**
 * @brief The ref module listens to a port for referee packets over the network.
 *
 * @details Referee packets are sent out from the
 * [ssl-refbox](https://github.com/Hawk777/ssl-refbox) program.
 * You can see the [protobuf
 * packet](https://github.com/Hawk777/ssl-refbox/blob/master/referee.proto) for
 * full details,
 * but the packets contains info about which stage of the game it is, team
 * scores, yellow/red cards, etc.
 *
 * Each time a new packet arrives, the ref module updates the GameState object
 * with the new information.
 */
class Referee : public Node {
public:
    explicit Referee(Context* const context);
    ~Referee() override = default;

    Referee(const Referee&) = delete;
    Referee& operator=(const Referee&) = delete;
    Referee(Referee&&) = delete;
    Referee& operator=(Referee&&) = delete;

    void start() override;
    void run() override;

    [[nodiscard]] bool kicked() const { return _kickDetectState == Kicked; }

    [[nodiscard]] bool isBlueTeam() const {
        return _context->game_state.blueTeam;
    }

protected:
    RefereeModuleEnums::Stage stage_;
    RefereeModuleEnums::Command command_;

    // The UNIX timestamp when the packet was sent, in microseconds.
    // Divide by 1,000,000 to get a time_t.
    RJ::Time sent_time;
    RJ::Time received_time;

    // The number of microseconds left in the stage.
    // The following stages have this value; the rest do not:
    // NORMAL_FIRST_HALF
    // NORMAL_HALF_TIME
    // NORMAL_SECOND_HALF
    // EXTRA_TIME_BREAK
    // EXTRA_FIRST_HALF
    // EXTRA_HALF_TIME
    // EXTRA_SECOND_HALF
    // PENALTY_SHOOTOUT_BREAK
    //
    // If the stage runs over its specified time, this value
    // becomes negative.
    RJ::Seconds stage_time_left_;

    // The number of commands issued since startup (mod 2^32).
    int64_t command_counter;

    // The UNIX timestamp when the command was issued, in microseconds.
    // This value changes only when a new command is issued, not on each packet.
    RJ::Time command_timestamp;

    TeamInfo yellow_info;
    TeamInfo blue_info;

    [[nodiscard]] GameState updateGameState(
        RefereeModuleEnums::Command command) const;

    void update();

    // Unconditional setter for the team color.
    void blueTeam(bool value) { _context->game_state.blueTeam = value; }
    void spinKickWatcher(const SystemState& system_state);

    enum KickDetectState {
        WaitForReady,
        CapturePosition,
        WaitForKick,
        VerifyKick,
        Kicked
    };
    KickDetectState _kickDetectState;

    Geometry2d::Point _readyBallPos;

    // Time the ball was first beyond KickThreshold from its original position
    RJ::Time _kickTime;

    std::mutex _mutex;
    Context* const _context;

    RefereeModuleEnums::Command prev_command_;
    RefereeModuleEnums::Stage prev_stage_;

    // Whether or not WE are currently controlled by the ref. This is not the
    // same as whether the referee is connected, because it will still be false
    // if the ref is connected but our team name doesn't match either of the
    // team names in the packet.
    bool _isRefControlled = false;

    float ballPlacementX;
    float ballPlacementY;

private:
    GameState _game_state;

    // Arbitrary receive buffer size
    static constexpr size_t RecvBufferSize =
        std::numeric_limits<uint16_t>::max() + 1;
    std::array<char, RecvBufferSize> _recv_buffer;

    boost::asio::io_service _io_service;
    boost::asio::ip::udp::socket _asio_socket;
    boost::asio::ip::udp::endpoint _sender_endpoint;

    void setupRefereeMulticast();
    void startReceive();
    void receivePacket(const boost::system::error_code& error,
                       size_t num_bytes);
};
