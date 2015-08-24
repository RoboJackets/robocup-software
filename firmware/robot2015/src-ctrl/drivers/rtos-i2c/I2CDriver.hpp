#pragma once

#include "stdint.h"
#include "I2C.h"
#include "Mutex.h"

#include "DigitalOut.h"


namespace mbed
{
/// I2C driver based on mbed RTOS and I2C-C-API.
/// Supports Master and Slave mode
class I2CDriver
{
public:
    //static DigitalOut osci2;
    /// Status returned by the receiveSlave() function
    enum SlaveRxStatus {
        NoData         = 0,
        ReadAddressed  = 1,
        WriteGeneral   = 2,
        WriteAddressed = 3
    };

    /** Create an I2C Master interface, connected to the specified pins.
     *
     *  @param sda I2C data line pin
     *  @param scl I2C clock line pin
     *
     *  @note Has to be created in a thread context, i.e. within the main or some other function. A global delaration does not work
     */
    I2CDriver(PinName sda, PinName scl, int hz=100000, int slaveAdr=0);

    /** Set the frequency of the I2C interface
    *
    *  @param hz The bus frequency in hertz
    */
    void frequency(int hz) {
        m_freq = hz;
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
    int readMaster(int address, char* data, int length, bool repeated = false);

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
    int readMaster(int address, uint8_t _register, char* data, int length, bool repeated = false);

    /** Read a single byte from the I2C bus
     *
     *  @param ack indicates if the byte is to be acknowledged (1 = acknowledge)
     *
     *  @returns
     *    the byte read
     */
    int readMaster(int ack=1);

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
    int writeMaster(int address, const char *data, int length, bool repeated = false);

    /** Write single byte out on the I2C bus
     *
     *  @param data data to write out on bus
     *
     *  @returns
     *    '1' if an ACK was received,
     *    '0' otherwise
     */
    int writeMaster(int data);

    /** Sets the I2C slave address.
     *
     *  @param address The address to set for the slave (ignoring the least
     *  signifcant bit). If set to 0, the slave will only respond to the
     *  general call address.
     */
    void addressSlave(int address) {
        m_slaveAdr=(address & 0xff) | 1;
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
    int receiveSlave(uint32_t timeout_ms=osWaitForever);

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
    int readSlave(char *data, int length);

    /** Read a single byte from an I2C master.
    *
    *  @returns
    *    the byte read
    */
    int readSlave(void);

    /** Write to an I2C master.
     *
     *  @param data pointer to the byte array to be transmitted
     *  @param length the number of bytes to transmite
     *
     *  @returns
     *       0 on success,
     *   non-0 otherwise
     */
    int writeSlave(const char *data, int length);

    /** Write a single byte to an I2C master.
    *
    *  @data the byte to write
    *
    *  @returns
    *    '1' if an ACK was received,
    *    '0' otherwise
    */
    int writeSlave(int data);


    /// Creates a start condition on the I2C bus
    void startMaster(void);

    ///Creates a stop condition on the I2C bus
    void stopSlave(void);

    /// Creates a stop condition on the I2C bus
    /// If unsccessful because someone on the bus holds the scl line down it returns "false" after 23µs
    /// In normal operation the stop shouldn't take longer than 12µs @ 100kHz and 3-4µs @ 400kHz.
    bool stopMaster(void);

    /// Wait until the i2c driver becomes available.
    ///
    /// Useful if you want to run a sequence of command without interrution by another thread.
    /// There's no need to call this function for running single request, because all driver functions
    /// will lock the device for exclusive access automatically.
    void lock();

    /// Unlock the driver that has previously been locked by the same thread.
    void unlock();

protected:
    void config();
    void lockNconfig() {
        lock();
        config();
    }

    // structure that holds I2C channels status
    struct Channel {
        rtos::Mutex mutex;
        i2c_t i2c;
        int freq;
        int slaveAdr;
        bool modeSlave;
        bool initialized;
        osThreadId callerID;
        osPriority callerPrio;
    };

    // current i2c configuration of this driver interface
    int m_freq;
    int m_slaveAdr;
    bool m_modeSlave;

    // i2c driver prio
    static const osPriority c_drvPrio = osPriorityRealtime;
    // the pin names fo the i2c channels
    static const PinName c_sdas[2];
    static const PinName c_scls[2];

    // static storage for the I2C channel access objects
    static Channel* s_channels[2];

    // i2c channel object of this driver interface, in fact just a pointer
    /// to one of the entries in s_channels
    Channel* m_channel;
};
}
