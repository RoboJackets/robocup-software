#pragma once

#include <cstdint>

#include <boost/asio.hpp>

#include "context.hpp"
#include "radio.hpp"

namespace radio {

/**
 * @brief Radio IO with robots in the simulator
 */
class SimRadio : public Radio {
public:
    static std::size_t instance_count;
    SimRadio(bool blue_team = false);

protected:
    void send(int robot_id, const rj_msgs::msg::MotionSetpoint& motion,
              const rj_msgs::msg::ManipulatorSetpoint& manipulator) override;
    void receive() override;
    void switch_team(bool blue) override;

private:
    void stop_robots();

    void handle_receive(const std::string& data);
    void start_receive();
    void receive_packet(const boost::system::error_code& error, size_t num_bytes);

    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint grsim_endpoint_;

    std::vector<char> buffer_;

    bool blue_team_;
};

}  // namespace radio
