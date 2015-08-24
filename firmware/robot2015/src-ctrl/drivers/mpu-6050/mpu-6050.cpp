/**
 * Includes
 */
#include "mpu-6050.hpp"

#include <logger.hpp>


MPU6050::MPU6050(PinName sda, PinName scl) : connection(sda, scl, 400000)
{
    setSleepMode(false);
    //Initializations:
    currentGyroRange = 0;
    currentAcceleroRange = 0;
}

//--------------------------------------------------
//-------------------General------------------------
//--------------------------------------------------

void MPU6050::write(uint8_t address, uint8_t data)
{
    uint8_t temp[2];
    temp[0] = address;
    temp[1] = data;
    connection.write(MPU6050_ADDRESS * 2, (char*)temp, 2);
}

uint8_t MPU6050::read(uint8_t address)
{
    uint8_t retval;
    connection.write(MPU6050_ADDRESS * 2, (char*)&address, 1, true);
    connection.read(MPU6050_ADDRESS * 2, (char*)&retval, 1);
    return retval;
}

void MPU6050::read(uint8_t address, uint8_t* data, int length)
{
    connection.write(MPU6050_ADDRESS * 2, (char*)&address, 1, true);
    connection.read(MPU6050_ADDRESS * 2, (char*)data, length);
}

void MPU6050::setSleepMode(bool state)
{
    uint8_t temp;
    temp = this->read(MPU6050_RA_PWR_MGMT_1);

    if (state == true)
        temp |= 1 << MPU6050_SLP_BIT;

    if (state == false)
        temp &= ~(1 << MPU6050_SLP_BIT);

    this->write(MPU6050_RA_PWR_MGMT_1, temp);
}

bool MPU6050::testConnection(void)
{
    uint8_t temp;
    temp = this->read(MPU6050_RA_WHO_AM_I);
    LOG(OK, "MPU-6050 'WHO_AM_I' reg: 0x%02X", temp);
    return (temp == (MPU6050_ADDRESS & 0xFE));
}

void MPU6050::setBW(uint8_t BW)
{
    uint8_t temp;
    BW = BW & 0x07;
    temp = this->read(MPU6050_RA_CONFIG);
    temp &= 0xF8;
    temp = temp + BW;
    this->write(MPU6050_RA_CONFIG, temp);
}

uint8_t MPU6050::getRate(void)
{
    return this->read(MPU6050_RA_SMPLRT_DIV);
}

void MPU6050::setI2CBypass(bool state)
{
    uint8_t temp;
    temp = this->read(MPU6050_RA_INT_PIN_CFG);

    if (state == true)
        temp |= 1 << MPU6050_BYPASS_BIT;

    if (state == false)
        temp &= ~(1 << MPU6050_BYPASS_BIT);

    this->write(MPU6050_RA_INT_PIN_CFG, temp);
}

//--------------------------------------------------
//----------------Accelerometer---------------------
//--------------------------------------------------

void MPU6050::setAcceleroRange(uint8_t range)
{
    uint8_t temp;
    range = range & 0x03;
    currentAcceleroRange = range;
    temp = this->read(MPU6050_RA_ACCEL_CONFIG);
    temp &= ~(3 << 3);
    temp = temp + (range << 3);
    this->write(MPU6050_RA_ACCEL_CONFIG, temp);
}

int MPU6050::getAcceleroRawX(void)
{
    short retval;
    uint8_t data[2];
    this->read(MPU6050_RA_ACCEL_XOUT_H, data, 2);
    retval = (data[0] << 8) + data[1];
    return (int) retval;
}

int MPU6050::getAcceleroRawY(void)
{
    short retval;
    uint8_t data[2];
    this->read(MPU6050_RA_ACCEL_YOUT_H, data, 2);
    retval = (data[0] << 8) + data[1];
    return (int) retval;
}

int MPU6050::getAcceleroRawZ(void)
{
    short retval;
    uint8_t data[2];
    this->read(MPU6050_RA_ACCEL_ZOUT_H, data, 2);
    retval = (data[0] << 8) + data[1];
    return (int) retval;
}

void MPU6050::getAcceleroRaw(int* data)
{
    uint8_t temp[6];
    this->read(MPU6050_RA_ACCEL_XOUT_H, temp, 6);
    data[0] = (int)(short)((temp[0] << 8) + temp[1]);
    data[1] = (int)(short)((temp[2] << 8) + temp[3]);
    data[2] = (int)(short)((temp[4] << 8) + temp[5]);
}

void MPU6050::getAccelero(float* data)
{
    int temp[3];
    this->getAcceleroRaw(temp);

    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_2G) {
        data[0] = (float) temp[0] / 16384.0 * 9.81;
        data[1] = (float) temp[1] / 16384.0 * 9.81;
        data[2] = (float) temp[2] / 16384.0 * 9.81;
    }

    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_4G) {
        data[0] = (float) temp[0] / 8192.0 * 9.81;
        data[1] = (float) temp[1] / 8192.0 * 9.81;
        data[2] = (float) temp[2] / 8192.0 * 9.81;
    }

    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_8G) {
        data[0] = (float) temp[0] / 4096.0 * 9.81;
        data[1] = (float) temp[1] / 4096.0 * 9.81;
        data[2] = (float) temp[2] / 4096.0 * 9.81;
    }

    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_16G) {
        data[0] = (float) temp[0] / 2048.0 * 9.81;
        data[1] = (float) temp[1] / 2048.0 * 9.81;
        data[2] = (float) temp[2] / 2048.0 * 9.81;
    }
}

//--------------------------------------------------
//------------------Gyroscope-----------------------
//--------------------------------------------------
void MPU6050::setGyroRange(uint8_t range)
{
    uint8_t temp;
    currentGyroRange = range;
    range = range & 0x03;
    temp = this->read(MPU6050_RA_GYRO_CONFIG);
    temp &= ~(3 << 3);
    temp = (temp + range) << 3;
    this->write(MPU6050_RA_GYRO_CONFIG, temp);
}

int MPU6050::getGyroRawX(void)
{
    short retval;
    uint8_t data[2];
    this->read(MPU6050_RA_GYRO_XOUT_H, data, 2);
    retval = (data[0] << 8) + data[1];
    return (int) retval;
}

int MPU6050::getGyroRawY(void)
{
    short retval;
    uint8_t data[2];
    this->read(MPU6050_RA_GYRO_YOUT_H, data, 2);
    retval = (data[0] << 8) + data[1];
    return (int) retval;
}

int MPU6050::getGyroRawZ(void)
{
    short retval;
    uint8_t data[2];
    this->read(MPU6050_RA_GYRO_ZOUT_H, data, 2);
    retval = (data[0] << 8) + data[1];
    return (int) retval;
}

void MPU6050::getGyroRaw(int* data)
{
    uint8_t temp[6];
    this->read(MPU6050_RA_GYRO_XOUT_H, temp, 6);
    data[0] = (int)(short)((temp[0] << 8) + temp[1]);
    data[1] = (int)(short)((temp[2] << 8) + temp[3]);
    data[2] = (int)(short)((temp[4] << 8) + temp[5]);
}

void MPU6050::getGyro(float* data)
{
    int temp[3];
    this->getGyroRaw(temp);

    if (currentGyroRange == MPU6050_GYRO_RANGE_250) {
        data[0] = (float) temp[0] / 7505.7;
        data[1] = (float) temp[1] / 7505.7;
        data[2] = (float) temp[2] / 7505.7;
    }

    if (currentGyroRange == MPU6050_GYRO_RANGE_500) {
        data[0] = (float) temp[0] / 3752.9;
        data[1] = (float) temp[1] / 3752.9;
        data[2] = (float) temp[2] / 3752.9;
    }

    if (currentGyroRange == MPU6050_GYRO_RANGE_1000) {
        data[0] = (float) temp[0] / 1879.3;;
        data[1] = (float) temp[1] / 1879.3;
        data[2] = (float) temp[2] / 1879.3;
    }

    if (currentGyroRange == MPU6050_GYRO_RANGE_2000) {
        data[0] = (float) temp[0] / 939.7;
        data[1] = (float) temp[1] / 939.7;
        data[2] = (float) temp[2] / 939.7;
    }
}

//--------------------------------------------------
//-------------------Temperature--------------------
//--------------------------------------------------
int MPU6050::getTempRaw(void)
{
    short retval;
    uint8_t data[2];
    this->read(MPU6050_RA_TEMP_OUT_H, data, 2);
    retval = (data[0] << 8) + data[1];
    return (int) retval;
}

float MPU6050::getTemp(void)
{
    float retval;
    retval = (float)this->getTempRaw();
    retval = (retval + 521.0) / 340.0 + 35.0;
    return retval;
}

//--------------------------------------------------
//--------------------Self Test---------------------
//--------------------------------------------------
// TODO: This needs some work...
bool MPU6050::selfTest(void)
{
    uint8_t buf[4];
    uint8_t gyroSTen[3];
    uint8_t accelSTen[3];
    uint8_t gyroSTdis[3];
    uint8_t accelSTdis[3];
    uint8_t gyroSTR[3];
    uint8_t accelSTR[3];
    float gyroFT[3];
    float accelFT[3];

    // get the currently set gyro/accel ranges so we can reset it before we return
    uint8_t tempGyro = this->read(MPU6050_RA_GYRO_CONFIG) & (0x03 << 3);
    uint8_t tempAccel = this->read(MPU6050_RA_GYRO_CONFIG) & (0x03 << 3);

    // force the gyro range to +-250 (accel range to +-8) and enable
    // self-test for X, Y, & Z accel/gyro.
    this->write(MPU6050_RA_GYRO_CONFIG, (0x07 << 5));
    this->write(MPU6050_RA_ACCEL_CONFIG, (0x1E << 3));

    // Read all self test values for gyro and accel
    this->read(MPU6050_RA_SELF_TEST_X, buf, 3);
    buf[3] = read(MPU6050_RA_SELF_TEST_A);

    for (int i = 0; i < 3; i++) {
        gyroSTen[i] = buf[i] & 0x1F;
        accelSTen[i] = (buf[i] >> 3) | (buf[3] >> (2 * (2 - i)));
    }

    // Disable accel/gyro self-test mode and read the output values again
    this->write(MPU6050_RA_GYRO_CONFIG, (0x00 << 5));
    this->write(MPU6050_RA_ACCEL_CONFIG, (0x00 << 5));

    // Read all self test values for gyro and accel
    this->read(MPU6050_RA_SELF_TEST_X, buf, 3);
    buf[3] = read(MPU6050_RA_SELF_TEST_A);

    for (int i = 0; i < 3; i++) {
        gyroSTdis[i] = buf[i] & 0x1F;
        accelSTdis[i] = (buf[i] >> 3) | (buf[3] >> (2 * (2 - i)));
    }

    // compute the self-test response values
    for (int i = 0; i < 3; i++) {
        gyroSTR[i] = gyroSTen[i] - gyroSTdis[i];
        accelSTR[i] = accelSTen[i] - accelSTdis[i];
    }

    // Reset the accel/gyro ranges to whatever they were before this function was called
    this->write(MPU6050_RA_GYRO_CONFIG, tempGyro);
    this->write(MPU6050_RA_ACCEL_CONFIG, tempAccel);

    // compute the factory trim values for the accel/gyro
    genGyroFT(gyroSTdis, gyroFT);
    genAccelFT(accelSTdis, accelFT);

    // compute the percentages of the self-test results
    for (int i = 0; i < 3; i++) {
        gyroSTR[i] = (gyroSTR[i] - gyroFT[i]) / gyroFT[i];
        accelSTR[i] = (accelSTR[i] - accelFT[i]) / accelFT[i];
    }

    LOG(OK,
        "Gyro self-test results:\r\n"
        "  X:\t%01.2f\r\n"
        "  Y:\t%01.2f\r\n"
        "  Z:\t%01.2f\r\n"
        "Accel self-test results:\r\n"
        "  X:\t%01.2f\r\n"
        "  Y:\t%01.2f\r\n"
        "  Z:\t%01.2f\r\n",
        gyroFT[0],
        gyroFT[1],
        gyroFT[2],
        accelFT[0],
        accelFT[1],
        accelFT[2]
       );

    return true;
}

// Must pass arrays of length 3
void MPU6050::genGyroFT(uint8_t* vals, float* results)
{
    float multiplier = 3275;
    float base = 1.046;

    // implicit conversion to float from uint8_t
    for (size_t i = 0; i < 3; i++) {
        if (results[i] == 0)
            results[i] = 0;
        else
            results[i] = (-1 * (i + 2)) * multiplier * pow(base, vals[i] - 1);
    }
}

// Must pass arrays of length 3
void MPU6050::genAccelFT(uint8_t* vals, float* results)
{
    float multiplier = 1392.64;
    float base = 0.92 / 0.34;
    uint8_t divider = 30;

    // implicit conversion to float from uint8_t
    for (size_t i = 0; i < 3; i++) {
        if (results[i] == 0)
            results[i] = 0;
        else
            results[i] = multiplier * pow(base, (vals[i] - 1) / divider);
    }
}
