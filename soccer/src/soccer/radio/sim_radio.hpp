#pragma once

#include <cstdint>

#include <boost/asio.hpp>

#include <rj_common/team_color.hpp>

#include "context.hpp"
#include "radio.hpp"
#include "system_state.hpp"

namespace radio {

/**
 * @brief Radio IO with robots in the simulator
 */
class SimRadio : public Radio {
public:
    SimRadio(TeamColor team = TeamColor::kBlue);

protected:
    void send(RobotId robot_id, const rj_msgs::msg::MotionSetpoint& motion,
              const rj_msgs::msg::ManipulatorSetpoint& manipulator) override;
    void receive() override;
    void switch_team(TeamColor team) override;

private:
    void stop_robots();

    void handle_receive(const std::string& data);
    void start_receive();
    void receive_packet(const boost::system::error_code& error, size_t num_bytes);

    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint grsim_endpoint_;

    std::vector<char> buffer_;

    TeamColor team_;
};

}  // namespace radio
