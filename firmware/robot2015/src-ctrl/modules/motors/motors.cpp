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
    .desc = "Motor"
};
}

std::vector<motor_t> motors(NUM_MOTORS, mtrEx);
int start_s = clock();

void motors_Init() {
    // not sure why this sometimes causes a hard fault...
    // this will be changed anyways once motors are working
    motors.at(0).desc += "1";
    motors.at(1).desc += "2";
    motors.at(2).desc += "3";
    motors.at(3).desc += "4";
    motors.at(4).desc = "Dribb.";
}

void motors_show() {
    std::array<uint16_t, NUM_MOTORS> duty_cycles = {0};
    std::array<uint8_t, NUM_MOTORS> halls = {0};
    std::array<uint16_t, NUM_MOTORS> enc_deltas = {0};

    FPGA::Instance()->read_duty_cycles(duty_cycles.data(), duty_cycles.size());
    FPGA::Instance()->read_halls(halls.data(), halls.size());
    FPGA::Instance()->read_encs(enc_deltas.data(), enc_deltas.size());

    // The status byte fields:
    //   { sys_rdy, watchdog_trigger, motors_en, is_connected[4:0] }
    uint8_t status_byte = FPGA::Instance()->watchdog_reset();

    printf("%s", (ansi::reset_font + ansi::cursor_hide).c_str());
    printf("Status:%s\t\t%s%s", ansi::clear_to_end.c_str(),
           status_byte & 0x20 ? "ENABLED" : "DISABLED", ansi::row_next_start.c_str());
    printf("Last Update:%s\t%.2fms\t%s%s", ansi::clear_to_end.c_str(),
           (static_cast<float>(enc_deltas.back()) * (1 / 18.432) * 63) / 1000,
           status_byte & 0x40 ? "[EXPIRED]" : "[OK]", ansi::row_next_start.c_str());
    printf("%s    ID\t\tVEL\tHALL\tENC\tDIR\tSTATUS%s", ansi::clear_to_end.c_str(), ansi::row_next_start.c_str());
    for (size_t i = 0; i < duty_cycles.size() - 1; i++) {
        printf("%s    %s\t%-u\t%-3u\t%-5u\t%s\t%s%s",
               ansi::clear_to_end.c_str(),
               motors.at(i).desc.c_str(), duty_cycles.at(i) & 0x1FF,
               halls.at(i), enc_deltas.at(i),
               duty_cycles.at(i) & (1 << 9) ? "CW" : "CCW",
               (status_byte & (1 << i)) ? "[OK]" : "[UNCONN]",
               ansi::row_next_start.c_str());
    }
    printf(
        "%s    %s\t%-u\t%-3u\tN/A\t%s\t%s%s", motors.back().desc.c_str(),
        ansi::clear_to_end.c_str(),
        duty_cycles.back() & 0x1FF, halls.back(),
        duty_cycles.back() & (1 << 9) ? "CW" : "CCW",
        (status_byte & (1 << (enc_deltas.size() - 1))) ? "[OK]" : "[UNCONN]",
        ansi::row_next_start.c_str());
}

int cmd_motors_scroll(const std::vector<std::string>& args) {
    motors_show();
    // move cursor back 8 rows
    printf("\033[%uA%s", 8, ansi::cursor_show.c_str());
    Console::Flush();
    Thread::wait(350);
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
        motors_show();
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
                printf("%s velocity set to %u (%s)\r\n",
                       motors.at(motor_id).desc.c_str(), new_vel & 0x1FF,
                       new_vel & (1 << 9) ? "CW" : "CCW");
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
