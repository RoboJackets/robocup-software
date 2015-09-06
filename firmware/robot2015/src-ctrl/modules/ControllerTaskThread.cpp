#include "controller.hpp"
#include "TaskSignals.hpp"

#include <rtos.h>
#include <RPCVariable.h>
#include <logger.hpp>

#include "motors.hpp"
#include "mpu-6050.hpp"


// Keep this pretty high for now. Ideally, drop it down to ~3 for production builds. Hopefully that'll be possible without the console
#define CONTROL_LOOP_WAIT_MS 20


// Declaration for an alternative control loop thread for when the accel/gyro can't be used for whatever reason
void Task_Controller_Sensorless(void const* args);


namespace {
// The gyro/accel values are given RPC read/write access here
float gyroVals[3] = { 0 };
float accelVals[3] = { 0 };
RPCVariable<float> gyrox(&gyroVals[0], "gyro-x");
RPCVariable<float> gyroy(&gyroVals[1], "gyro-y");
RPCVariable<float> gyroz(&gyroVals[2], "gyro-z");
RPCVariable<float> accelx(&accelVals[0], "accel-x");
RPCVariable<float> accely(&accelVals[1], "accel-y");
RPCVariable<float> accelz(&accelVals[2], "accel-z");
}


/**
 * initializes the motion controller thread
 */
void Task_Controller(void const* args)
{
	// Store the thread's ID
	osThreadId threadID = Thread::gettid();

	// Store our priority so we know what to reset it to if ever needed
	osPriority threadPriority;

	if (threadID != nullptr)
		threadPriority  = osThreadGetPriority(threadID);
	else
		threadPriority = osPriorityIdle;

#if RJ_MPU_EN
	MPU6050 imu(RJ_I2C_BUS);

	imu.setBW(MPU6050_BW_256);
	imu.setGyroRange(MPU6050_GYRO_RANGE_250);
	imu.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);
	imu.setSleepMode(false);

	char testResp;
	if ( (testResp = imu.testConnection()) ) {
		LOG(INIT, "Control loop ready!\r\n    Thread ID:\t%u\r\n    Priority:\t%d", threadID, threadPriority);

	} else {
		LOG(SEVERE, "MPU6050 not found!\t(response: 0x%02X)\r\n    Falling back to sensorless control loop.", testResp);
		// TODO: Turn on the IMU's error LED here

#else
	LOG(INIT, "IMU disabled in config file\r\n    Falling back to sensorless control loop.");
#endif
		// Start a thread that can function without the IMU, then terminate this thread
		Thread controller_task(Task_Controller_Sensorless, nullptr, osPriorityRealtime);
		osThreadTerminate(threadID);

		return;

#if RJ_MPU_EN
	}

#endif

	osThreadSetPriority(threadID, osPriorityNormal);

	while (true)
	{
		imu.getGyro(gyroVals);
		imu.getAccelero(accelVals);

		LOG(INF2,
		    "Gyro:\t"
		    "(% 1.2f, % 1.2f, % 1.2f)\r\n"
		    "Accel:\t"
		    "(% 1.2f, % 1.2f, % 1.2f)",
		    gyroVals[0],
		    gyroVals[1],
		    gyroVals[2],
		    accelVals[0],
		    accelVals[1],
		    accelVals[2]
		   );

		Thread::wait(CONTROL_LOOP_WAIT_MS);
		Thread::yield();
	}

	osThreadTerminate(threadID);
}


/**
 * [Task_Controller_Sensorless]
 * @param args [description]
 */
void Task_Controller_Sensorless(void const* args)
{
	// Store the thread's ID
	osThreadId threadID = Thread::gettid();

	// Store our priority so we know what to reset it to if ever needed
	osPriority threadPriority;

	if (threadID != nullptr)
		threadPriority  = osThreadGetPriority(threadID);
	else
		threadPriority = osPriorityIdle;

	LOG(INIT, "Sensorless control loop ready!\r\n    Thread ID:\t%u\r\n    Priority:\t%d", threadID, threadPriority);

	while (1) {
		Thread::wait(CONTROL_LOOP_WAIT_MS);
		Thread::yield();
	}

	osThreadTerminate(threadID);
}
