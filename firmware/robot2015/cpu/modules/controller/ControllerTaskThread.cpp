#include "controller.hpp"

/**
 * initializes the motion controller thread
 */
void Task_Controller(void const* args)
{
	// Store the thread's ID
	osThreadId threadID = Thread::gettid();

	float gyroVals[3] = { 0 };
	float accelVals[3] = { 0 };

	MPU6050 imu(RJ_I2C_BUS);

	imu.setBW(MPU6050_BW_256);
	imu.setGyroRange(MPU6050_GYRO_RANGE_250);
	imu.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);
	imu.setSleepMode(false);
	// imu.selfTest();

	if (imu.testConnection()) {
		LOG(OK, "Controller ready! Thread ID: %u", threadID);
	} else {
		LOG(SEVERE, "MPU6050 not found!");
		// If there's no IMU chip, no need to continue running the thread
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

		Thread::wait(1000);
		Thread::yield();
	}

	osThreadTerminate(threadID);
}
