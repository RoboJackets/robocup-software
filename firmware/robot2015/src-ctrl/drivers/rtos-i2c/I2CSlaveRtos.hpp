#pragma once

#include "I2CDriver.hpp"

namespace mbed
{

/// I2C slave interface to the RTOS-I2CDriver.
/// The interface is compatible to the original mbed I2C class.
class I2CSlaveRtos
{
    I2CDriver m_drv;

public:
    /// Status returned by the receiveSlave() function
    enum RxStatus {
        NoData         = 0,
        ReadAddressed  = 1,
        WriteGeneral   = 2,
        WriteAddressed = 3
    };

    /** Create an I2C Slave interface, connected to the specified pins.
     *
     *  @param sda I2C data line pin
     *  @param scl I2C clock line pin
     *
     *  @note Has to be created in a thread context, i.e. within the main or some other function. A global delaration does not work
     */
    I2CSlaveRtos(PinName sda, PinName scl, int freq = 100000, int address = 42)
        : m_drv(sda, scl, 100000, address) {}

    /** Set the frequency of the I2C interface
     *
     *  @param hz The bus frequency in hertz
     */
    void frequency(int hz) {
        m_drv.frequency(hz);
    }

    /** Checks to see if this I2C Slave has been addressed.
     *
     *  @returns
     *  A status indicating if the device has been addressed, and how
     *  - NoData            - the slave has not been addressed
     *  - ReadAddressed     - the master has requested a read from this slave
     *  - WriteAddressed    - the master is writing to this slave
     *  - WriteGeneral      - the master is writing to all slave
     */
    int receive(uint32_t timeout_ms = osWaitForever) {
        return m_drv.receiveSlave(timeout_ms);
    }

    /** Read from an I2C master.
     *
     *  @param data pointer to the byte array to read data in to
     *  @param length maximum number of bytes to read
     *
     *  @returns
     *       0 on success,
     *   non-0 otherwise
     * ... no! instead it returns number of bytes read minus one ... weird, guess its a bug in the official lib
     */
    int read(char *data, int length) {
        return m_drv.readSlave(data, length);
    }

    /** Read a single byte from an I2C master.
     *
     *  @returns
     *    the byte read
     */
    int read() {
        return m_drv.readSlave();
    }

    /** Write to an I2C master.
     *
     *  @param data pointer to the byte array to be transmitted
     *  @param length the number of bytes to transmite
     *
     *  @returns
     *       0 on success,
     *   non-0 otherwise
     */
    int write(const char *data, int length) {
        return m_drv.writeSlave(data, length);
    }

    /** Write a single byte to an I2C master.
     *
     *  @data the byte to write
     *
     *  @returns
     *    '1' if an ACK was received,
     *    '0' otherwise
     */
    int write(int data) {
        return m_drv.writeSlave(data);
    }

    /** Sets the I2C slave address.
     *
     *  @param address The address to set for the slave (ignoring the least
     *  signifcant bit). If set to 0, the slave will only respond to the
     *  general call address.
     */
    void address(int address) {
        m_drv.addressSlave(address);
    }


    /** Reset the I2C slave back into the known ready receiving state.
     */
    void stop() {
        m_drv.stopSlave();
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
