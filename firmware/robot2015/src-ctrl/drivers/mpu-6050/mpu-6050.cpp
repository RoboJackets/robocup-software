
/**
 * Includes
 */
#include "mpu-6050.hpp"

#include <cmath>

#include <rtos.h>
#include <logger.hpp>


MPU6050::MPU6050(PinName sda, PinName scl, int freq) : connection(sda, scl, freq)
{
    setSleepMode(false);
    //Initializations:
    currentGyroRange = 0;
    currentAcceleroRange = 0;
}

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

bool MPU6050::testConnection()
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

uint8_t MPU6050::getRate()
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

int MPU6050::getAcceleroRawX()
{
    short retval;
    uint8_t data[2];
    this->read(MPU6050_RA_ACCEL_XOUT_H, data, 2);
    retval = (data[0] << 8) + data[1];
    return (int) retval;
}

int MPU6050::getAcceleroRawY()
{
    short retval;
    uint8_t data[2];
    this->read(MPU6050_RA_ACCEL_YOUT_H, data, 2);
    retval = (data[0] << 8) + data[1];
    return (int) retval;
}

int MPU6050::getAcceleroRawZ()
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

int MPU6050::getGyroRawX()
{
    short retval;
    uint8_t data[2];
    this->read(MPU6050_RA_GYRO_XOUT_H, data, 2);
    retval = (data[0] << 8) + data[1];
    return (int) retval;
}

int MPU6050::getGyroRawY()
{
    short retval;
    uint8_t data[2];
    this->read(MPU6050_RA_GYRO_YOUT_H, data, 2);
    retval = (data[0] << 8) + data[1];
    return (int) retval;
}

int MPU6050::getGyroRawZ()
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
int MPU6050::getTempRaw()
{
    short retval;
    uint8_t data[2];
    this->read(MPU6050_RA_TEMP_OUT_H, data, 2);
    retval = (data[0] << 8) + data[1];
    return (int) retval;
}

float MPU6050::getTemp()
{
    float retval;
    retval = (float)this->getTempRaw();
    retval = (retval + 521.0) / 340.0 + 35.0;
    return retval;
}


/* Should return percent deviation from factory trim
 * values, +/- 14 or less deviation is a pass.
 */
void MPU6050::selfTest(float* results)
{
    uint8_t rawData[4] = {0, 0, 0, 0};
    uint8_t selfTest[6];
    float factoryTrim[6];

    // Configure the accelerometer for self-test
    write(MPU6050_RA_ACCEL_CONFIG, 0xF0);      // Enable self test on all three axes and set accelerometer range to +/- 8 g
    write(MPU6050_RA_GYRO_CONFIG,  0xE0);      // Enable self test on all three axes and set gyro range to +/- 250 degrees/s

    // Delay a while to let the device execute the self-test
    Thread::wait(0.25);

    rawData[0] = read(MPU6050_RA_SELF_TEST_X); // X-axis self-test results
    rawData[1] = read(MPU6050_RA_SELF_TEST_Y); // Y-axis self-test results
    rawData[2] = read(MPU6050_RA_SELF_TEST_Z); // Z-axis self-test results
    rawData[3] = read(MPU6050_RA_SELF_TEST_A); // Mixed-axis self-test results

    // Extract the acceleration test results first
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ;    // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ;    // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ;    // ZA_TEST result is a five-bit unsigned integer

    // Extract the gyration test results first
    selfTest[3] = rawData[0] & 0x1F ;                               // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1] & 0x1F ;                               // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2] & 0x1F ;                               // ZG_TEST result is a five-bit unsigned integer

    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0f * 0.34f) * (std::pow( (0.92f / 0.34f), ((selfTest[0] - 1.0f) / 30.0f)));  // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0f * 0.34f) * (std::pow( (0.92f / 0.34f), ((selfTest[1] - 1.0f) / 30.0f)));  // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0f * 0.34f) * (std::pow( (0.92f / 0.34f), ((selfTest[2] - 1.0f) / 30.0f)));  // FT[Za] factory trim calculation
    factoryTrim[3] = (+25.0f * 131.0f) * (std::pow( 1.046f, (selfTest[3] - 1.0f) ));                    // FT[Xg] factory trim calculation
    factoryTrim[4] = (-25.0f * 131.0f) * (std::pow( 1.046f, (selfTest[4] - 1.0f) ));                    // FT[Yg] factory trim calculation
    factoryTrim[5] = (+25.0f * 131.0f) * (std::pow( 1.046f, (selfTest[5] - 1.0f) ));                    // FT[Zg] factory trim calculation

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        results[i] = 100.0f + 100.0f * (selfTest[i] - factoryTrim[i]) / factoryTrim[i]; // Report percent differences
    }
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU6050::calibrate(float * dest1, float * dest2)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
    const uint16_t gyrosensitivity  = 131;    // = 131 LSB/degrees/sec
    const uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

    // reset device, reset all registers, clear gyro and accelerometer bias registers
    write(MPU6050_RA_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    Thread::wait(0.1);

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    write(MPU6050_RA_PWR_MGMT_1, 0x01);
    write(MPU6050_RA_PWR_MGMT_2, 0x00);
    Thread::wait(0.2);

    // Configure device for bias calculation
    write(MPU6050_RA_INT_ENABLE,   0x00);  // Disable all interrupts
    write(MPU6050_RA_FIFO_EN,      0x00);  // Disable FIFO
    write(MPU6050_RA_PWR_MGMT_1,   0x00);  // Turn on internal clock source
    write(MPU6050_RA_I2C_MST_CTRL, 0x00);  // Disable I2C master
    write(MPU6050_RA_USER_CTRL,    0x00);  // Disable FIFO and I2C master modes
    write(MPU6050_RA_USER_CTRL,    0x0C);  // Reset FIFO and DMP
    Thread::wait(0.015);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    write(MPU6050_RA_CONFIG,       0x01);  // Set low-pass filter to 188 Hz
    write(MPU6050_RA_SMPLRT_DIV,   0x00);  // Set sample rate to 1 kHz
    write(MPU6050_RA_GYRO_CONFIG,  0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    write(MPU6050_RA_ACCEL_CONFIG, 0x00);  // Set accelerometer full-scale to 2 g, maximum sensitivity

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    write(MPU6050_RA_USER_CTRL, 0x40);     // Enable FIFO
    write(MPU6050_RA_FIFO_EN,   0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    Thread::wait(0.08); // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    write(MPU6050_RA_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO

    // read FIFO sample count
    read(MPU6050_RA_FIFO_COUNTH, &data[0], 2);
    fifo_count = ((uint16_t)data[0] << 8) | data[1];

    // How many sets of full gyro and accelerometer data for averaging
    packet_count = fifo_count / 12;

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};

        read(MPU6050_RA_FIFO_R_W, &data[0], 12); // read data for averaging

        // Form signed 16-bit integer for each sample in FIFO
        accel_temp[0] = (int16_t) (((int16_t)data[0]  << 8) | data[1]  );
        accel_temp[1] = (int16_t) (((int16_t)data[2]  << 8) | data[3]  );
        accel_temp[2] = (int16_t) (((int16_t)data[4]  << 8) | data[5]  );
        gyro_temp[0]  = (int16_t) (((int16_t)data[6]  << 8) | data[7]  );
        gyro_temp[1]  = (int16_t) (((int16_t)data[8]  << 8) | data[9]  );
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11] );

        // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[0] += (int32_t) accel_temp[0];
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];

    }

    // Normalize sums to get average count biases
    accel_bias[0] /= (int32_t) packet_count;
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    // Remove gravity from the z-axis accelerometer bias calculation
    if (accel_bias[2] > 0L) {
        accel_bias[2] -= (int32_t) accelsensitivity;
    } else {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4)       & 0xFF;
    data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4)       & 0xFF;

    // Push gyro biases to hardware registers
    write(MPU6050_RA_XG_OFFS_USRH, data[0]);
    write(MPU6050_RA_XG_OFFS_USRL, data[1]);
    write(MPU6050_RA_YG_OFFS_USRH, data[2]);
    write(MPU6050_RA_YG_OFFS_USRL, data[3]);
    write(MPU6050_RA_ZG_OFFS_USRH, data[4]);
    write(MPU6050_RA_ZG_OFFS_USRL, data[5]);

    // construct gyro bias in deg/s for later manual subtraction
    dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
    dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    read(MPU6050_RA_XA_OFFS_H, &data[0], 2); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    read(MPU6050_RA_YA_OFFS_H, &data[0], 2);
    accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    read(MPU6050_RA_ZA_OFFS_H, &data[0], 2);
    accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++) {
        if (accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Push accelerometer biases to hardware registers
    //  write(XA_OFFSET_H, data[0]);
    //  write(XA_OFFSET_L_TC, data[1]);
    //  write(YA_OFFSET_H, data[2]);
    //  write(YA_OFFSET_L_TC, data[3]);
    //  write(ZA_OFFSET_H, data[4]);
    //  write(ZA_OFFSET_L_TC, data[5]);

    // Output scaled accelerometer biases for manual subtraction in the main program
    dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
    dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
    dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
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
