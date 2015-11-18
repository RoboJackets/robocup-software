/**
 * RoboJackets: RoboCup SSL Firmware
 *
 * Copyright (C) 2015 RoboJackets JJ
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * motors.hpp - BLDC data structures.
 */

#pragma once

// Includes
#include <cstdint>
#include <array>
#include <vector>

/* Any math using the delta values for the hall/encoder (we really
 * only care about encoder readings...) is best handled through a task
 * queue where the calculations can be managed on the kernel's scheduler.
 * So we'll leave the MOTOR_MAX_SAMPLES at a minimum in the struct.
 */
#define MOTOR_MAX_SAMPLES 2

typedef uint16_t motorVel_t;

struct motorErr_t {
    /*
    Available Flags:
    =================
    GVDD_OV
    GVDD_UV
    PVDD_UV
    OTSD
    OTW
    FETHA_OC
    FETLA_OC
    FETHB_OC
    FETLB_OC
    FETHC_OC
    FETLC_OC
    */
    bool encOK;
    bool hallOK;
    std::array<uint16_t, 2> drvStatus;
};

struct motor_t {
    motorVel_t targetVel;
    motorVel_t adjVel;
    uint16_t hallCount;
    std::array<uint32_t, MOTOR_MAX_SAMPLES> encCounts;
    motorErr_t status;
    std::string desc;
};

void motors_Init();
void motors_PrintMotor(motor_t&);
void motors_cmdProcess(const std::vector<std::string>& args);
void motors_cmdScroll(const std::vector<std::string>& args);
