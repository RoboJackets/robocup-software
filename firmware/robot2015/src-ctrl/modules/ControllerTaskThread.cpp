#include <rtos.h>
#include <RPCVariable.h>

#include <Console.hpp>
#include <logger.hpp>
#include <assert.hpp>

#include "robot-devices.hpp"
#include "task-signals.hpp"
#include "motors.hpp"
#include "fpga.hpp"
#include "mpu-6050.hpp"
#include "io-expander.hpp"
#include "Pid.hpp"

const float kpi = 3.14159265358979f;

// Keep this pretty high for now. Ideally, drop it down to ~3 for production
// builds. Hopefully that'll be possible without the console
static const int CONTROL_LOOP_WAIT_MS = 5;

// Declaration for an alternative control loop thread for when the accel/gyro
// can't be used for whatever reason
void Task_Controller_Sensorless(const osThreadId mainThreadId);

namespace {
// The gyro/accel values are given RPC read/write access here
float gyroVals[3] = {0};
float accelVals[3] = {0};

// RPCVariable<float> gyrox(&gyroVals[0], "gyro-x");
// RPCVariable<float> gyroy(&gyroVals[1], "gyro-y");
// RPCVariable<float> gyroz(&gyroVals[2], "gyro-z");
// RPCVariable<float> accelx(&accelVals[0], "accel-x");
// RPCVariable<float> accely(&accelVals[1], "accel-y");
// RPCVariable<float> accelz(&accelVals[2], "accel-z");

// Making a temporary variable to test out the writing side of RPC variables
// int testVar;
// RPCVariable<int> test_var(&testVar, "var1");
}

/**
 * initializes the motion controller thread
 */
void Task_Controller(void const* args) {
    const osThreadId mainID = (const osThreadId)args;

    // Store the thread's ID
    osThreadId threadID = Thread::gettid();
    ASSERT(threadID != nullptr);

    // Store our priority so we know what to reset it to after running a command
    osPriority threadPriority = osThreadGetPriority(threadID);

    MPU6050 imu(RJ_I2C_SDA, RJ_I2C_SCL);

    imu.setBW(MPU6050_BW_256);
    imu.setGyroRange(MPU6050_GYRO_RANGE_250);
    imu.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);
    imu.setSleepMode(false);

    char testResp;
    if ((testResp = imu.testConnection())) {
        float resultRatio[6];
        imu.selfTest(resultRatio);
        LOG(INIT,
            "IMU self test results:\r\n"
            "    Accel (X,Y,Z):\t(%2.2f%%, %2.2f%%, %2.2f%%)\r\n"
            "    Gyro  (X,Y,Z):\t(%2.2f%%, %2.2f%%, %2.2f%%)",
            resultRatio[0], resultRatio[1], resultRatio[2], resultRatio[3],
            resultRatio[4], resultRatio[5]);

        LOG(INIT, "Control loop ready!\r\n    Thread ID: %u, Priority: %d",
            ((P_TCB)threadID)->task_id, threadPriority);
    } else {
        LOG(SEVERE,
            "MPU6050 not found!\t(response: 0x%02X)\r\n    Falling back to "
            "sensorless control loop.",
            testResp);

        // Start a thread that can function without the IMU, terminate us if it
        // ever returns
        Task_Controller_Sensorless(mainID);

        return;
    }

    // signal back to main and wait until we're signaled to continue
    osSignalSet(mainID, MAIN_TASK_CONTINUE);
    Thread::signal_wait(SUB_TASK_CONTINUE, osWaitForever);

    const uint16_t ENC_TICKS_PER_TURN = 2048;

    Pid motor2pid(0.0, 0.0, 0.0);

    std::vector<uint16_t> duty_cycles;

    const uint16_t kduty_cycle = 0;
    duty_cycles.assign(5, kduty_cycle);

    size_t ii = 0;
    bool spin_rev = true;

    uint16_t duty_cycle_all = kduty_cycle;

    while (true) {
        imu.getGyro(gyroVals);
        imu.getAccelero(accelVals);

        std::vector<uint16_t> enc_deltas(5);

        FPGA::Instance->set_duty_get_enc(duty_cycles.data(), duty_cycles.size(),
                                         enc_deltas.data(),
                                         enc_deltas.capacity());

        /*
         * The time since the last update is derived with the value of
         * WATCHDOG_TIMER_CLK_WIDTH in robocup.v
         *
         * The last encoder reading (5th one) from the FPGA is the watchdog
         * timer's tick since the last SPI transfer.
         *
         * Multiply the received tick count by:
         *     (1/18.432) * 2 * (2^WATCHDOG_TIMER_CLK_WIDTH)
         *
         * This will give you the duration since the last SPI transfer in
         * microseconds (us).
         *
         * For example, if WATCHDOG_TIMER_CLK_WIDTH = 6, here's how you would
         * convert into time assuming the fpga returned a reading of 1265 ticks:
         *     time_in_us = [ 1265 * (1/18.432) * 2 * (2^6) ] = 8784.7us
         *
         * The precision would be in increments of the multiplier. For
         * this example, that is:
         *     time_precision = 6.94us
         *
         */
        const float kdt = enc_deltas.back() * (1 / 18.432e6) * 2 * 64;

        // the target rev/s
        const float ktarget_rps = 5;

        // angular velocity of motor 3 in rad/s
        const float kvel = 2 * kpi * (enc_deltas[2] / ENC_TICKS_PER_TURN) / kdt;

        const float ktarget_vel = (2 * kpi) * ktarget_rps;

        // @125 duty cycle, 1260rpm @ no load
        TODO(remeasure the duty cycle and rad / s relationship of the motor)
        const float kmultiplier = 125.0f / (1260.0f * 2 * kpi / 60);

        const float vel_err = ktarget_vel - kvel;

        uint16_t dc = ktarget_vel * kmultiplier + motor2pid.run(vel_err);

        // duty cycle values range: 0 -> 511, the 9th bit is direction
        dc = std::min(dc, static_cast<uint16_t>(511));

        ii++;
        // if (ii < 20) {
        //     duty_cycle_all = kduty_cycle;
        // } else {
        //     duty_cycle_all = 0;
        //     spin_rev = !spin_rev;
        //     ii = 0;
        // }

        // if ((ii % 100) == 0) printf("dc: %u, dt: %f\r\n", dc, kdt);

        // set the direction
        duty_cycle_all |= (spin_rev << 9);

        std::fill(duty_cycles.begin(), duty_cycles.end(), duty_cycle_all);

        Thread::wait(CONTROL_LOOP_WAIT_MS);
    }
}

void Task_Controller_Sensorless(const osThreadId mainID) {
    // Store the thread's ID
    osThreadId threadID = Thread::gettid();
    ASSERT(threadID != nullptr);

    // Store our priority so we know what to reset it to after running a command
    osPriority threadPriority = osThreadGetPriority(threadID);

    LOG(INIT,
        "Sensorless control loop ready!\r\n    Thread ID: %u, Priority: %d",
        ((P_TCB)threadID)->task_id, threadPriority);

    // signal back to main and wait until we're signaled to continue
    osSignalSet(mainID, MAIN_TASK_CONTINUE);
    Thread::signal_wait(SUB_TASK_CONTINUE, osWaitForever);

    while (true) {
        Thread::wait(CONTROL_LOOP_WAIT_MS);
        Thread::yield();
    }
}
