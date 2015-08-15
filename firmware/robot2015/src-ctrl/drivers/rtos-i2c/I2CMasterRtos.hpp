#pragma once

#include "I2CDriver.hpp"

namespace mbed
{

/// I2C master interface to the RTOS-I2CDriver.
/// The interface is compatible to the original mbed I2C class.
/// Provides an additonal "read from register"-function.
class I2CMasterRtos
{
    I2CDriver m_drv;

public:
    /** Create an I2C Master interface, connected to the specified pins
    *
    *  @param sda I2C data line pin
    *  @param scl I2C clock line pin
    *
    *  @note Has to be created in a thread context, i.e. within the main or some other function. A global delaration does not work
    */
    I2CMasterRtos(PinName sda, PinName scl, int freq = 100000):m_drv(sda,scl,freq) {}

    /** Set the frequency of the I2C interface
     *
     *  @param hz The bus frequency in hertz
     */
    void frequency(int hz) {
        m_drv.frequency(hz);
    }

    /** Read from an I2C slave
     *
     * Performs a complete read transaction. The bottom bit of
     * the address is forced to 1 to indicate a read.
     *
     *  @param address 8-bit I2C slave address [ addr | 1 ]
     *  @param data Pointer to the byte-array to read data in to
     *  @param length Number of bytes to read
     *  @param repeated Repeated start, true - don't send stop at end
     *
     *  @returns
     *       0 on success (ack),
     *   non-0 on failure (nack)
     */
    int read(int address, char *data, int length = 1, bool repeated = false) {
        return m_drv.readMaster( address, data, length, repeated);
    }

    /** Read from a given I2C slave register
    *
    * Performs a complete write-register-read-data-transaction. The bottom bit of
    * the address is forced to 1 to indicate a read.
    *
    *  @param address 8-bit I2C slave address [ addr | 1 ]
    *  @param _register 8-bit regster address
    *  @param data Pointer to the byte-array to read data in to
    *  @param length Number of bytes to read
    *  @param repeated Repeated start, true - don't send stop at end
    *
    *  @returns
    *       0 on success (ack),
    *   non-0 on failure (nack)
    */
    int read(int address, uint8_t _register, char* data, int length = 1, bool repeated = false) {
        return m_drv.readMaster( address, _register, data, length, repeated);
    }

    /** Read a single byte from the I2C bus
     *
     *  @param ack indicates if the byte is to be acknowledged (1 = acknowledge)
     *
     *  @returns
     *    the byte read
     */
    int read(int ack) {
        return m_drv.readMaster(ack);
    }

    /** Write to an I2C slave
     *
     * Performs a complete write transaction. The bottom bit of
     * the address is forced to 0 to indicate a write.
     *
     *  @param address 8-bit I2C slave address [ addr | 0 ]
     *  @param data Pointer to the byte-array data to send
     *  @param length Number of bytes to send
     *  @param repeated Repeated start, true - do not send stop at end
     *
     *  @returns
     *       0 on success (ack),
     *   non-0 on failure (nack)
     */
    int write(int address, const char *data, int length = 1, bool repeated = false) {
        return m_drv.writeMaster(address, data, length, repeated);
    }

    /** Write single byte out on the I2C bus
     *
     *  @param data data to write out on bus
     *
     *  @returns
     *    '1' if an ACK was received,
     *    '0' otherwise
     */
    int write(int data) {
        return m_drv.writeMaster(data);
    }

    /** Creates a start condition on the I2C bus
     */

    void start(void) {
        m_drv.startMaster();
    }

    /// Creates a stop condition on the I2C bus
    /// If unsccessful because someone on the bus holds the scl line down it returns "false" after 23µs 
    /// In normal operation the stop shouldn't take longer than 12µs @ 100kHz and 3-4µs @ 400kHz. 
    bool stop(void) {
        return m_drv.stopMaster();
    }

    /// Wait until the interface becomes available.
    ///
    /// Useful if you want to run a sequence of command without interrution by another thread.
    /// There's no need to call this function for running single request, because all driver functions
    /// will lock the device for exclusive access automatically.
    void lock() {
        m_drv.lock();
    }

    /// Unlock the interface that has previously been locked by the same thread.
    void unlock() {
        m_drv.unlock();
    }

};
}
