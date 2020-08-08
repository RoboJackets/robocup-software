#pragma once

#include <Context.hpp>
#include <SystemState.hpp>
#include <boost/asio.hpp>
#include <cstdint>

#include "Radio.hpp"

/**
 * @brief Radio IO with robots in the simulator
 */
class SimRadio : public Radio {
public:
    static std::size_t instance_count;
    SimRadio(Context* context, bool blue_team = false);

    [[nodiscard]] bool is_open() const override;
    void send(const std::array<RobotIntent, kNumShells>& intents,
              const std::array<MotionSetpoint, kNumShells>& setpoints) override;
    void receive() override;
    void switch_team(bool blue_team) override;

    void stop_robots();

private:
    Context* const context_;

    void handle_receive(const std::string& data);
    void start_receive();
    void receive_packet(const boost::system::error_code& error,
                       size_t num_bytes);

    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint grsim_endpoint_;

    std::vector<char> buffer_;

    bool blue_team_;
};
