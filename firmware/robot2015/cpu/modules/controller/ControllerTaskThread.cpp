#include "controller.hpp"

/**
 * initializes the motion controller thread
 */
void Task_Controller(void const *args)
{
	float gyroVals[3] = { 0 };
	float accelVals[3] = { 0 };
	MPU6050 imu(RJ_I2C_BUS);

	imu.setBW(MPU6050_BW_256);
	imu.setGyroRange(MPU6050_GYRO_RANGE_250);
	imu.setAcceleroRange(MPU6050_ACCELERO_RANGE_2G);
	imu.setSleepMode(false);

	if (imu.testConnection())
		LOG(OK, "Controller ready!");
	else
		LOG(SEVERE, "MPU6050 not found!");

	while (true) {
		imu.getGyro(gyroVals);
		imu.getAccelero(accelVals);

		Thread::wait(1000);
		LOG(INF1, "Gyro updated.");
		// Thread::yield();
	}
}
