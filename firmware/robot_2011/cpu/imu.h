#pragma once

extern int imu_aligned;

extern float linear_acceleration[3];
// extern float quaternion[4];

int imu_init();
void imu_update();
