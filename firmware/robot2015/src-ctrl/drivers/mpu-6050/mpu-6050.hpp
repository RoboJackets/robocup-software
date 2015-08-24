#pragma once

/**
 * Includes
 */
#include <cstdint>
#include <mbed.h>

#include "mpu-6050-defines.hpp"
#include "I2CMasterRtos.hpp"
 

/**
 * Defines
 */
#ifndef MPU6050_ADDRESS
#define MPU6050_ADDRESS             0x69 // address pin low (GND), default for InvenSense evaluation board
#endif

/**
 * Definitions
 */
#define MPU6050_SLP_BIT             6
#define MPU6050_BYPASS_BIT          1

#define MPU6050_BW_256              0
#define MPU6050_BW_188              1
#define MPU6050_BW_98               2
#define MPU6050_BW_42               3
#define MPU6050_BW_20               4
#define MPU6050_BW_10               5
#define MPU6050_BW_5                6

#define MPU6050_ACCELERO_RANGE_2G   0
#define MPU6050_ACCELERO_RANGE_4G   1
#define MPU6050_ACCELERO_RANGE_8G   2
#define MPU6050_ACCELERO_RANGE_16G  3

#define MPU6050_GYRO_RANGE_250      0
#define MPU6050_GYRO_RANGE_500      1
#define MPU6050_GYRO_RANGE_1000     2
#define MPU6050_GYRO_RANGE_2000     3


/** MPU6050 IMU library.
  *
  * Example:
  * @code
  * Later, maybe
  * @endcode
  */
class MPU6050
{
  public:
     /**
     * Constructor.
     *
     * Sleep mode of MPU6050 is immediatly disabled
     *
     * @param sda - mbed pin to use for the SDA I2C line.
     * @param scl - mbed pin to use for the SCL I2C line.
     */
     MPU6050(PinName sda, PinName scl);


     /**
     * Tests the I2C connection by reading the WHO_AM_I register.
     *
     * @return True for a working connection, false for an error
     */
     bool testConnection(void);

     /**
     * Sets the bandwidth of the digital low-pass filter
     *
     * Macros: MPU6050_BW_256 - MPU6050_BW_188 - MPU6050_BW_98 - MPU6050_BW_42 - MPU6050_BW_20 - MPU6050_BW_10 - MPU6050_BW_5
     * Last number is the gyro's BW in Hz (accelero BW is virtually identical)
     *
     * @param BW - The three bits that set the bandwidth (use the predefined macros)
     */
     void setBW(uint8_t BW);

     /**
     * Sets the auxiliary I2C bus in bypass mode to read the sensors behind the MPU6050 (useful for eval board, otherwise just connect them to primary I2C bus)
     *
     * @param state - Enables/disables the I2C bypass mode
     */
     void setI2CBypass(bool state);

     /**
     * Sets the Accelero full-scale range
     *
     * Macros: MPU6050_ACCELERO_RANGE_2G - MPU6050_ACCELERO_RANGE_4G - MPU6050_ACCELERO_RANGE_8G - MPU6050_ACCELERO_RANGE_16G
     *
     * @param range - The two bits that set the full-scale range (use the predefined macros)
     */
     void setAcceleroRange(uint8_t range);

     /**
     * Reads the accelero x-axis.
     *
     * @return 16-bit signed integer x-axis accelero data
     */
     int getAcceleroRawX(void);

     /**
     * Reads the accelero y-axis.
     *
     * @return 16-bit signed integer y-axis accelero data
     */
     int getAcceleroRawY(void);

     /**
     * Reads the accelero z-axis.
     *
     * @return 16-bit signed integer z-axis accelero data
     */
     int getAcceleroRawZ(void);

     /**
     * Reads all accelero data.
     *
     * @param data - pointer to signed integer array with length three: data[0] = X, data[1] = Y, data[2] = Z
     */
     void getAcceleroRaw(int* data);

     /**
     * Reads all accelero data, gives the acceleration in m/s2
     *
     * Function uses the last setup value of the full scale range, if you manually set in another range, this won't work.
     *
     * @param data - pointer to float array with length three: data[0] = X, data[1] = Y, data[2] = Z
     */
     void getAccelero(float* data);

     /**
     * Sets the Gyro full-scale range
     *
     * Macros: MPU6050_GYRO_RANGE_250 - MPU6050_GYRO_RANGE_500 - MPU6050_GYRO_RANGE_1000 - MPU6050_GYRO_RANGE_2000
     *
     * @param range - The two bits that set the full-scale range (use the predefined macros)
     */
     void setGyroRange(uint8_t range);

     /**
     * Reads the gyro x-axis.
     *
     * @return 16-bit signed integer x-axis gyro data
     */
     int getGyroRawX(void);

     /**
     * Reads the gyro y-axis.
     *
     * @return 16-bit signed integer y-axis gyro data
     */
     int getGyroRawY(void);

     /**
     * Reads the gyro z-axis.
     *
     * @return 16-bit signed integer z-axis gyro data
     */
     int getGyroRawZ(void);

     /**
     * Reads all gyro data.
     *
     * @param data - pointer to signed integer array with length three: data[0] = X, data[1] = Y, data[2] = Z
     */
     void getGyroRaw(int* data);

     /**
     * Reads all gyro data, gives the gyro in rad/s
     *
     * Function uses the last setup value of the full scale range, if you manually set in another range, this won't work.
     *
     * @param data - pointer to float array with length three: data[0] = X, data[1] = Y, data[2] = Z
     */
     void getGyro(float* data);

     /**
     * Reads temperature data.
     *
     * @return 16 bit signed integer with the raw temperature register value
     */
     int getTempRaw(void);

     /**
     * Returns current temperature
     *
     * @returns float with the current temperature
     */
     float getTemp(void);

     uint8_t getRate(void);

     /**
     * Sets the sleep mode of the MPU6050
     *
     * @param state - true for sleeping, false for wake up
     */
     void setSleepMode(bool state);


     /**
     * Writes data to the device, could be private, but public is handy so you can transmit directly to the MPU.
     *
     * @param adress - register address to write to
     * @param data - data to write
     */
     void write(uint8_t address, uint8_t data);

     /**
     * Read data from the device, could be private, but public is handy so you can transmit directly to the MPU.
     *
     * @param adress - register address to write to
     * @return - data from the register specified by RA
     */
     uint8_t read(uint8_t adress);

     /**
     * Read multtiple regigsters from the device, more efficient than using multiple normal reads.
     *
     * @param adress - register address to write to
     * @param length - number of bytes to read
     * @param data - pointer where the data needs to be written to
     */
     void read(uint8_t adress, uint8_t* data, int length);

     bool selfTest(void);

  private:
     I2CMasterRtos connection;
     uint8_t currentAcceleroRange;
     uint8_t currentGyroRange;

     void genGyroFT(uint8_t*, float*);
     void genAccelFT(uint8_t*, float*);
};
