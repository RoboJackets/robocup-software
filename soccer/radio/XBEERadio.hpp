#pragma once

#include "Radio.hpp"

class XBEERadio : public Radio {
public:
    XBEERadio();
    ~XBEERadio();

    virtual bool isOpen() const override;
    virtual void send(Packet::RadioTx& packet) override;
    virtual void receive() override;

    virtual void channel(int n) override;
    void switchTeam(bool) override {}

protected:
    std::vector<DebugCommunication::DebugResponse> current_recieve_debug;
    std::mutex current_recieve_debug_mutex;

    bool open();

    void command(uint8_t cmd);
    void write(uint8_t reg, uint8_t value);
    uint8_t read(uint8_t reg);
};