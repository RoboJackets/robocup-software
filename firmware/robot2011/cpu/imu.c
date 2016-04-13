#include "imu.h"

#include "invensense/imuSetup.h"
#include "invensense/imuMlsl.h"
#include "invensense/imuFIFO.h"
#include "invensense/imuMldl.h"
#include "invensense/mpuregs.h"

#include <stdio.h>
#include <limits.h>

int imu_aligned;
float linear_acceleration[3];

static void motionCallback(uint16_t motionType) {
    if (motionType == ML_NO_MOTION) {
        imu_aligned = 1;
    }
}

int imu_init() {
    imu_aligned = 0;

    if (IMUopen() != ML_SUCCESS) {
        return 0;
    }

    MLSetAuxSlaveAddr(KIONIX_AUX_SLAVEADDR);

    // FIXME - Do this in fixed point to reduce bloat
    float gyroCal[9] = {0};
    float accelCal[9] = {0};
    float gyroScale = 0.0, accelScale = 0.0;
    // 0 3 6    0 1 0
    // 1 4 7   -1 0 0
    // 2 5 8    0 0 1
    gyroCal[1] = -1.0;
    gyroCal[3] = 1.0;
    gyroCal[8] = 1.0;
    accelCal[0] = -1.0;
    accelCal[4] = -1.0;
    accelCal[8] = 1.0;

    gyroScale = 2000.0f;
    accelScale = 2.0f;

    MLSetGyroCalibration(gyroScale, gyroCal);
    MLSetAccelCalibration(accelScale, accelCal);

    IMUsetBiasUpdateFunc(ML_ALL);
    IMUsendQuaternionToFIFO(ML_32_BIT);
    IMUsendGyroToFIFO(ML_ELEMENT_3, ML_32_BIT);
    IMUsendLinearAccelWorldToFIFO(ML_ALL, ML_32_BIT);
    IMUsendLinearAccelToFIFO(ML_ALL, ML_32_BIT);
    // 	MLSetProcessedFIFOCallback(dataCallback);
    IMUsetMotionCallback(motionCallback);
    IMUsetFIFORate(0);
    IMUstart();

    return 1;
}

void imu_update() {
    IMUupdateData();

    // read quaternion
    // units????
    float quaternion[4];
    IMUgetQuaternionFloat(quaternion);

    // read accelerations and convert int -> float.
    // units in linear_acceleration are in m/s^2
    long acc[3] = {0};
    IMUgetLinearAccelWorld(acc);
    const int SCALE_FACTOR = 65535;
    linear_acceleration[0] = (float)acc[0] / SCALE_FACTOR;
    linear_acceleration[1] = (float)acc[1] / SCALE_FACTOR;
    linear_acceleration[2] = (float)acc[2] / SCALE_FACTOR;

    // TODO: rotate linear acc using the quaternion
}
