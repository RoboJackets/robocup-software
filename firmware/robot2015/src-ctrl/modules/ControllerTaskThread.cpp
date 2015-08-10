#include "controller.hpp"

#include <rtos.h>
#include <logger.hpp>

#include "motors.hpp"
#include "mpu-6050.hpp"


// Keep this pretty high for now. Ideally, drop it down to ~3 for production builds. Hopefully that'll be possible without the console
#define CONTROL_LOOP_WAIT_MS 250


/**
 * initializes the motion controller thread
 */
void Task_Controller(void const* args)
{
	// Store the thread's ID
	osThreadId threadID = Thread::gettid();

	// Store our priority so we know what to reset it to if ever needed
	osPriority threadPriority;

	if (threadID != NULL)
		threadPriority  = osThreadGetPriority(threadID);
	else
		threadPriority = (osPriority)NULL;

	float gyroVals[3] = { 0 };
	float accelVals[3] = { 0 };

	MPU6050 imu(RJ_I2C_BUS);

	imu.setBW(MPU6050_BW_256);
	imu.setGyroRange(MPU6050_GYRO_RANGE_250);
	imu.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);
	imu.setSleepMode(false);
	// imu.selfTest();

	if (imu.testConnection()) {
		LOG(INIT, "Control loop ready!\r\n    Thread ID:\t%u\r\n    Priority:\t%d", threadID, threadPriority);

	} else {
		LOG(SEVERE, "MPU6050 not found!\r\n    Falling back to sensorless control loop.");

		// Once things are more organized, startup a sensorless control loop thread before killing this one.
		osThreadTerminate(threadID);
	}

	while (true) {
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
		//Thread::yield();
	}

	osThreadTerminate(threadID);
}
