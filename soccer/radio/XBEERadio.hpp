#pragma once

#include "Radio.hpp"
#include <xbee.h>

#include <mutex>

/**
 * @brief Radio with XBEE Series 1
 * 
 * @details This class allows us to communicate with XBEE Series 1 for 
 * robot control. Currently a work in progress. 
 */
class XBEERadio : public Radio {
    public:
        XBEERadio();
        XBEERadio(int id);
        XBEERadio(std::string usbport);
        ~XBEERadio();


        virtual bool isOpen() const override;
        virtual void send(Packet::RadioTx& packet) override;
        virtual void receive() override;

        virtual void channel(int n) override;
        void switchTeam(bool) override {}

        void testonly();

    protected:
        struct xbee *xbee;
        struct xbee_con *con;
        struct xbee_conAddress address;
        struct xbee_conSettings settings;
        xbee_err ret;

        bool open();

        void command(uint8_t cmd);
        void write(uint8_t reg, uint8_t value);
        uint8_t read(uint8_t reg);

    private:
        void send_broadcast(Packet::RadioTx& packet);
};