#include <RPCVariable.h>
#include <rtos.h>

#include <Console.hpp>
#include <assert.hpp>
#include <logger.hpp>

#include "PidMotionController.hpp"
#include "RtosTimerHelper.hpp"
#include "fpga.hpp"
#include "io-expander.hpp"
#include "motors.hpp"
#include "mpu-6050.hpp"
#include "robot-devices.hpp"
#include "task-signals.hpp"

using namespace std;

// Keep this pretty high for now. Ideally, drop it down to ~3 for production
// builds. Hopefully that'll be possible without the console
static const int CONTROL_LOOP_WAIT_MS = 5;

// initialize PID controller
PidMotionController pidController;

/** If this amount of time (in ms) elapses without
 * Task_Controller_UpdateTarget() being called, the target velocity is reset to
 * zero.  This is a safety feature to prevent robots from doing unwanted things
 * when they lose radio communication.
 */
static const uint32_t COMMAND_TIMEOUT_INTERVAL = 250;
unique_ptr<RtosTimerHelper> commandTimeoutTimer = nullptr;
bool commandTimedOut = true;

void Task_Controller_UpdateTarget(Eigen::Vector3f targetVel) {
    pidController.setTargetVel(targetVel);

    // reset timeout
    commandTimedOut = false;
    if (commandTimeoutTimer)
        commandTimeoutTimer->start(COMMAND_TIMEOUT_INTERVAL);
}

uint8_t dribblerSpeed = 0;
void Task_Controller_UpdateDribbler(uint8_t dribbler) {
    dribblerSpeed = dribbler;
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

    // MPU6050 imu(RJ_I2C_SDA, RJ_I2C_SCL);

    // imu.setBW(MPU6050_BW_256);
    // imu.setGyroRange(MPU6050_GYRO_RANGE_250);
    // imu.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);
    // imu.setSleepMode(false);

    // char testResp;
    // if ((testResp = imu.testConnection())) {
    //     float resultRatio[6];
    //     imu.selfTest(resultRatio);
    //     LOG(INIT,
    //         "IMU self test results:\r\n"
    //         "    Accel (X,Y,Z):\t(%2.2f%%, %2.2f%%, %2.2f%%)\r\n"
    //         "    Gyro  (X,Y,Z):\t(%2.2f%%, %2.2f%%, %2.2f%%)",
    //         resultRatio[0], resultRatio[1], resultRatio[2], resultRatio[3],
    //         resultRatio[4], resultRatio[5]);
    //
    //     LOG(INIT, "Control loop ready!\r\n    Thread ID: %u, Priority: %d",
    //         ((P_TCB)threadID)->task_id, threadPriority);
    // } else {
    //     LOG(SEVERE,
    //         "MPU6050 not found!\t(response: 0x%02X)\r\n    Falling back to "
    //         "sensorless control loop.",
    //         testResp);
    // }

    // signal back to main and wait until we're signaled to continue
    osSignalSet(mainID, MAIN_TASK_CONTINUE);
    Thread::signal_wait(SUB_TASK_CONTINUE, osWaitForever);

    array<int16_t, 5> duty_cycles{};

    // pidController.setPidValues(1.5, 0.05, 0);  // TODO: tune pid values
    pidController.setPidValues(0.2, 0, 0);

    // initialize timeout timer
    commandTimeoutTimer = make_unique<RtosTimerHelper>(
        [&]() { commandTimedOut = true; }, osTimerOnce);

    while (true) {
        // imu.getGyro(gyroVals);
        // imu.getAccelero(accelVals);

        // note: the 4th value is not an encoder value.  See the large comment
        // below for an explanation.
        array<int16_t, 5> enc_deltas{};

        // zero out command if we haven't gotten an updated target in a while
        if (commandTimedOut) duty_cycles = {0, 0, 0, 0, 0};

        FPGA::Instance->set_duty_get_enc(duty_cycles.data(), duty_cycles.size(),
                                         enc_deltas.data(), enc_deltas.size());

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
        const float dt = enc_deltas.back() * (1 / 18.432e6) * 2 * 64;

        // take first 4 encoder deltas
        array<int16_t, 4> driveMotorEnc;
        for (int i = 0; i < 4; i++) driveMotorEnc[i] = enc_deltas[i];

        // run PID controller to determine what duty cycles to use to drive the
        // motors.
        array<int16_t, 4> driveMotorDutyCycles =
            pidController.run(driveMotorEnc, dt);
        for (int i = 0; i < 4; i++) duty_cycles[i] = driveMotorDutyCycles[i];

        // limit duty cycle values, while keeping sign (+ or -)
        for (int16_t& dc : duty_cycles) {
            if (std::abs(dc) > FPGA::MAX_DUTY_CYCLE) {
                dc = copysign(FPGA::MAX_DUTY_CYCLE, dc);
            }
        }

        // dribbler duty cycle
        duty_cycles[4] = dribblerSpeed;

#if 0
        // log duty cycle values
        printf("duty cycles: ");
        for (int i = 0; i < 4; i++) {
            printf("%d, ", duty_cycles[i]);
        }
        printf("\r\n");
#endif

        Thread::wait(CONTROL_LOOP_WAIT_MS);
    }
}
