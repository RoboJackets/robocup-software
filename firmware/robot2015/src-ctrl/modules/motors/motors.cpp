#include "motors.hpp"

#include <mbed.h>
#include <rtos.h>
#include <Console.hpp>
#include <numparser.hpp>

#include "fpga.hpp"
#include "commands.hpp"

namespace {
const int NUM_MOTORS = 5;

motor_t mtrEx = {
    .vel = 0x4D,
    .hall = 0x0A,
    .enc = {0x23, 0x18},
    .status = {.encOK = false, .hallOK = true, .drvStatus = {0x26, 0x0F}},
    .desc = "Motor"};
}

std::vector<motor_t> motors(NUM_MOTORS, mtrEx);
int start_s = clock();

void motors_Init() {
    // not sure why this sometimes causes a hard fault...
    // this will be changed anyways once motors are working
    // motors.at(0).desc = "Dribb.";
    // motors.at(1).desc += "1";
    // motors.at(2).desc += "2";
    // motors.at(3).desc += "3";
    // motors.at(4).desc += "4";
}

int cmd_motors_scroll(const std::vector<std::string>& args) {
    std::array<uint16_t, NUM_MOTORS> duty_cycles = {0};
    std::array<uint8_t, NUM_MOTORS> halls = {0};
    std::array<uint16_t, NUM_MOTORS> enc_deltas = {0};

    FPGA::Instance()->read_duty_cycles(duty_cycles.data(), duty_cycles.size());
    FPGA::Instance()->read_halls(halls.data(), halls.size());
    uint8_t status_byte =
        FPGA::Instance()->read_encs(enc_deltas.data(), enc_deltas.size());

    // printf("\033[?25l\033[25mMotors Enabled: \033[K%s\033E",
    //        status_byte & 0x20 ? "YES" : "NO");
    printf("\033[?25l\033[25mStatus Byte: \033[K0x%02X\033E", status_byte);
    for (size_t i = 0; i < duty_cycles.size(); i++) {
        printf("\t%s \tVel: 0x%03X\tHall: %3u\tEnc: %5u%s\033E",
               motors.at(i).desc.c_str(), duty_cycles.at(i), halls.at(i),
               enc_deltas.at(i), (status_byte & (1 << i)) ? "FAULT" : "");
    }
    printf("\033[%uA\033[?25h", duty_cycles.size() + 1);
    Console::Flush();

    Thread::wait(315);
    return 0;
}

// The console function to run with the 'motor' command
int cmd_motors(const std::vector<std::string>& args) {
    if (args.empty() == true) {
        show_invalid_args(args);
        return 1;
    }

    else if (strcmp(args.front().c_str(), "on") == 0) {
        FPGA::Instance()->motors_en(true);
        printf("Motors enabled.\r\n");
    }

    else if (strcmp(args.front().c_str(), "off") == 0) {
        FPGA::Instance()->motors_en(false);
        printf("Motors disabled.\r\n");
    }

    else if (strcmp(args.front().c_str(), "show") == 0) {
        std::array<uint16_t, NUM_MOTORS> duty_cycles;
        std::array<uint8_t, NUM_MOTORS> halls;
        std::array<uint16_t, NUM_MOTORS> encs;

        FPGA::Instance()->read_duty_cycles(duty_cycles.data(),
                                           duty_cycles.size());
        FPGA::Instance()->read_halls(halls.data(), halls.size());
        uint8_t status_byte =
            FPGA::Instance()->read_encs(encs.data(), encs.size());

        for (size_t i = 0; i < duty_cycles.size(); i++) {
            printf("\t%s \tVel: 0x%03X\tHall: %3u\tEnc: %5u%s\r\n",
                   motors.at(i).desc.c_str(), duty_cycles.at(i), halls.at(i),
                   encs.at(i), (status_byte & (1 << i)) ? "FAULT" : "");
        }
    }

    else if (strcmp(args.front().c_str(), "set") == 0) {
        if (args.size() == 3) {
            std::array<uint16_t, NUM_MOTORS> duty_cycles;
            uint8_t motor_id = static_cast<uint8_t>(atoi(args.at(1).c_str()));
            uint16_t new_vel = static_cast<uint16_t>(atoi(args.at(2).c_str()));
            // return error if motor id doesn't exist
            if (motor_id > 4) return 2;
            // get the current duty cycles for all motors
            uint8_t status_byte = FPGA::Instance()->read_duty_cycles(
                duty_cycles.data(), duty_cycles.size());
            // change our specific motor's duty cycle and write all duty cycles
            // back to the FPGA
            if (status_byte != 0x7F) {
                duty_cycles.at(motor_id) = new_vel;
                FPGA::Instance()->set_duty_cycles(duty_cycles.data(),
                                                  duty_cycles.size());
                printf("%s duty cycle set to 0x%03X\r\n",
                       motors.at(motor_id).desc.c_str(), new_vel);
            } else {
                // return an error if an invalid duty cycle was given
                printf("Motor %u does not exist", motor_id);
                return 3;
            }
        } else {
            show_invalid_args(args);
            return 4;
        }
    }

    else {
        show_invalid_args(args);
        return 5;
    }

    return 0;
}
