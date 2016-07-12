#include "radio_protocol.h"
#include "radio.h"
#include "timer.h"
#include "status.h"
#include "fpga.h"
#include "ball_sense.h"
#include "power.h"
#include "encoder_monitor.h"
#include "stall.h"
#include "imu.h"
#include "invensense/imuFIFO.h"

#include <string.h>

#define Forward_Size 55

// Last forward packet
uint8_t forward_packet[Forward_Size];

int cmd_body_x;
int cmd_body_y;
int cmd_body_w;
int dribble_command;
int kick_command;
int kick_immediate;
int accel_limit;
int decel_limit;
int sing;
int anthem;

int sequence;

uint8_t reply_buf[64];
uint8_t reply_len;
uint32_t rx_lost_time;

void reply_timer_init() {
    // TC0 is used for reply timing.
    // Start the timer with the appropriate value in RC when a forward packet is
    // received.
    // The timer reaches RC and stops when the reverse packet needs to be sent.
    //
    // The reply timer runs at MCK/32 => 1.5MHz
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC0;
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN;
    AT91C_BASE_TC0->TC_CMR = AT91C_TC_WAVE | AT91C_TC_WAVESEL_UP_AUTO |
                             AT91C_TC_CPCSTOP | AT91C_TC_CLKS_TIMER_DIV3_CLOCK;
}

void reply_timer_start(uint16_t time) {
    if (time < 2) {
        time = 2;
    }

    AT91C_BASE_TC0->TC_RC = time - 1;
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_SWTRG;
}

void radio_reply() {
    if (reply_len && reply_timer_done()) {
        radio_transmit(reply_buf, reply_len);
        reply_len = 0;
    }
}

int motor_status(int i) {
    int bit = 1 << i;
    if (motor_faults & bit) {
        return 1;
    } else if (motor_stall & bit) {
        return 2;
    } else if (encoder_faults & bit) {
        return 3;
    } else {
        // Nothing is wrong
        return 0;
    }
}

int handle_forward_packet() {
    if (radio_rx_len != Forward_Size) {
        radio_command(SFRX);
        radio_command(SRX);
        return 0;
    }

    memcpy(forward_packet, radio_rx_buf, Forward_Size);

    rx_lost_time = current_time;
    LED_TOGGLE(LED_RG);
    LED_OFF(LED_RR);

    sequence = forward_packet[0] & 7;

    // Clear motor commands in case this robot's ID does not appear in the
    // packet
    cmd_body_x = 0;
    cmd_body_y = 0;
    cmd_body_w = 0;
    dribble_command = 0;

    // Get motor commands from the packet
    int offset = 1;
    int reply_slot = -1;
    for (int slot = 0; slot < 6; ++slot) {
        if ((forward_packet[offset + 4] & 0x0f) == robot_id) {
            uint8_t more = forward_packet[offset + 3];
            cmd_body_x = forward_packet[offset + 0] | ((more & 0x03) << 8);
            cmd_body_y = forward_packet[offset + 1] | ((more & 0x0c) << 6);
            cmd_body_w = forward_packet[offset + 2] | ((more & 0x30) << 4);

            // Sign extension
            if (cmd_body_x & 0x200) {
                cmd_body_x |= 0xfffffc00;
            }
            if (cmd_body_y & 0x200) {
                cmd_body_y |= 0xfffffc00;
            }
            if (cmd_body_w & 0x200) {
                cmd_body_w |= 0xfffffc00;
            }

            // Convert the dribbler speed from the top four bits in a byte to
            // nine bits
            uint8_t four_bits = forward_packet[offset + 4] & 0xf0;
            dribble_command =
                (four_bits << 1) | (four_bits >> 3) | (four_bits >> 7);

            kick_command = forward_packet[offset + 5];
            use_chipper = forward_packet[offset + 6] & 1;
            kick_immediate = forward_packet[offset + 6] & 2;
            accel_limit = forward_packet[offset + 7];
            decel_limit = forward_packet[offset + 7];
            sing = forward_packet[offset + 6] & 4;
            anthem = forward_packet[offset + 6] & 8;

            reply_slot = slot;
            break;
        }
        offset += 9;
    }

    if (accel_limit > 40) {
        accel_limit = 40;
    }

    if (decel_limit > 40) {
        decel_limit = 40;
    }

    if (reply_slot >= 0) {
        // Build and send a reverse packet
        reply_buf[0] = robot_id | (sequence << 4);
        reply_buf[1] = last_rssi;
        reply_buf[2] = supply_raw * VBATT_NUM / VBATT_DIV / 100;
        reply_buf[3] = kicker_status;
        if (failures & Fail_Kicker) {
            reply_buf[3] |= 0x80;
        }

        // Drive motor status
        reply_buf[4] = 0;
        for (int i = 0; i < 4; ++i) {
            reply_buf[4] |= motor_status(i) << (i * 2);
        }

        // Dribbler status
        reply_buf[5] = motor_status(4);

        // Ball sensor status
        if (failures &
            (Fail_Ball_LED_Open | Fail_Ball_Det_Open | Fail_Ball_Det_Short)) {
            reply_buf[5] |= 3 << 2;
        } else if (failures & Fail_Ball_Dazzled) {
            reply_buf[5] |= 2 << 2;
        } else if (have_ball) {
            reply_buf[5] |= 1 << 2;
        }

        // Mechanical version
        if (base2008) {
            reply_buf[5] |= 1 << 4;
        }

        if (imu_aligned) {
            reply_buf[5] |= 1 << 5;
        }

        reply_buf[6] = kicker_voltage;

        long quat[4] = {0};
        IMUgetQuaternion(quat);
        for (int i = 0; i < 4; ++i) {
            reply_buf[7 + i * 2] = quat[i] >> 16;
            reply_buf[8 + i * 2] = quat[i] >> 24;
        }

        reply_len = 7;
        reply_timer_start(3000 * reply_slot);
    } else {
        // Get ready to receive another forward packet
        radio_command(SRX);
    }

    return 1;
}
