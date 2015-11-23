#include "I2CDriver.hpp"

#include <logger.hpp>

#include "i2cRtos_api.hpp"

using namespace mbed;
using namespace rtos;

#define PREFIX i2cRtos
//#define PREFIX i2c // fallback to offical busy wait i2c c-api for performance
// testing
#define PASTER(x, y) x##_##y
#define EVALUATOR(x, y) PASTER(x, y)
#define FUNCTION(fun) EVALUATOR(PREFIX, fun)

const PinName I2CDriver::c_sdas[] = {p9, p28};
const PinName I2CDriver::c_scls[] = {p10, p27};

I2CDriver::Channel* I2CDriver::s_channels[2] = {0, 0};

I2CDriver::I2CDriver(PinName sda, PinName scl, int hz, int slaveAdr)
    : m_freq(hz), m_slaveAdr(slaveAdr) {
    // ensure exclusive access for initialization
    static Mutex mtx;
    bool locked = false;
    if (osKernelRunning()) {  // but don't try to lock if rtos kernel is not
                              // running yet. (global/static definition)
        mtx.lock();
        locked = true;
    }

    // check pins and determine i2c channel
    int channel = 0;
#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
    if (sda == c_sdas[0] && scl == c_scls[0])
        channel = 0;  // I2C_1
    else
#endif
        if (sda == c_sdas[1] && scl == c_scls[1])
        channel = 1;  // I2C_2 or I2C
    else
        LOG(SEVERE, "I2CDriver: Invalid I2C pins selected");

    // initialize the selected i2c channel
    if (s_channels[channel] == 0) {
        s_channels[channel] = new I2CDriver::Channel;
        m_channel = s_channels[channel];
        m_channel->freq = 0;
        m_channel->slaveAdr = 0;
        m_channel->modeSlave = 0;
        m_channel->initialized = false;  // defer i2c initialization util we are
                                         // sure the rtos kernel is running
                                         // (config() function)
    }
    m_channel = s_channels[channel];
    if (locked) mtx.unlock();
}

void I2CDriver::lock() {
    // osci2.write(1);
    // One and the same thread can lock twice, but then it needs also to unlock
    // twice.
    // exactly what we need here
    m_channel->mutex.lock(osWaitForever);
    m_channel->callerID = osThreadGetId();
    m_channel->callerPrio = osThreadGetPriority(m_channel->callerID);
    // maximize thread prio
    osThreadSetPriority(m_channel->callerID, c_drvPrio);  // hopefully not
                                                          // interrupted since
                                                          // the lock in the
                                                          // line above
    // mutex code looks like that waiting threads are priority ordered
    // also priority inheritance seems to be provided
    // osci2.write(0);
}

void I2CDriver::unlock() {
    // osci2.write(1);
    // free the mutex and restore original prio
    // rt_tsk_lock();  // just prevent beeing preempted after restoring prio
    // before freeing the mutex
    osThreadSetPriority(m_channel->callerID, m_channel->callerPrio);
    m_channel->mutex.unlock();
    // rt_tsk_unlock();
    // osci2.write(0);
}

void I2CDriver::config() {
    // osci2.write(1);
    // check and initialize driver
    if (!m_channel->initialized) {
        int channel = m_channel == s_channels[0] ? 0 : 1;  // ...ugly
        FUNCTION(init)(&m_channel->i2c, c_sdas[channel], c_scls[channel]);
        m_channel->initialized = true;
    }
    // check and update frequency
    if (m_freq != m_channel->freq) {
        m_channel->freq = m_freq;
        i2c_frequency(&m_channel->i2c, m_freq);
    }
    // check and update slave/master mode
    if (m_modeSlave != m_channel->modeSlave) {
        m_channel->modeSlave = m_modeSlave;
        i2c_slave_mode(&m_channel->i2c, m_modeSlave);
    }
    // check and update slave address
    if (m_modeSlave && m_slaveAdr != m_channel->slaveAdr) {
        m_channel->slaveAdr = m_slaveAdr;
        i2c_slave_address(&m_channel->i2c, 0, m_slaveAdr, 0);
    }
    // osci2.write(0);
}

int I2CDriver::readMaster(int address, char* data, int length, bool repeated) {
    m_modeSlave = false;
    lockNconfig();
    int ret = FUNCTION(read)(&m_channel->i2c, address, data, length,
                             (repeated ? 0 : 1));
    unlock();
    return ret;
}
int I2CDriver::readMaster(int address, uint8_t _register, char* data,
                          int length, bool repeated) {
    m_modeSlave = false;
    lockNconfig();
    int ret = FUNCTION(write)(&m_channel->i2c, address, (const char*)&_register,
                              1, 0);
    if (!ret)
        ret = FUNCTION(read)(&m_channel->i2c, address, data, length,
                             (repeated ? 0 : 1));
    unlock();
    return ret;
}
int I2CDriver::readMaster(int ack) {
    m_modeSlave = false;
    lockNconfig();
    int ret = i2cRtos_byte_read(&m_channel->i2c, (ack ? 0 : 1));
    unlock();
    return ret;
}
int I2CDriver::writeMaster(int address, const char* data, int length,
                           bool repeated) {
    m_modeSlave = false;
    lockNconfig();
    int ret = FUNCTION(write)(&m_channel->i2c, address, data, length,
                              (repeated ? 0 : 1));
    unlock();
    return ret;
}
int I2CDriver::writeMaster(int data) {
    m_modeSlave = false;
    lockNconfig();
    int ret = i2cRtos_byte_write(&m_channel->i2c, data);
    unlock();
    return ret;
}
void I2CDriver::startMaster() {
    m_modeSlave = false;
    lockNconfig();
    i2c_start(&m_channel->i2c);
    unlock();
}
bool I2CDriver::stopMaster() {
    m_modeSlave = false;
    lockNconfig();
    bool ret = i2cRtos_stop(&m_channel->i2c);
    unlock();
    return ret;
}
void I2CDriver::stopSlave() {
    m_modeSlave = true;
    lockNconfig();
    i2c_stop(&m_channel->i2c);
    unlock();
}
int I2CDriver::receiveSlave(uint32_t timeout_ms) {
    m_modeSlave = true;
    lockNconfig();
    int ret = i2cRtos_slave_receive(&m_channel->i2c, timeout_ms);
    unlock();
    return ret;
}
int I2CDriver::readSlave(char* data, int length) {
    m_modeSlave = true;
    lockNconfig();
    int ret = i2cRtos_slave_read(&m_channel->i2c, data, length);
    unlock();
    return ret;
}
int I2CDriver::readSlave() {
    m_modeSlave = true;
    lockNconfig();
    int ret = i2cRtos_byte_read(&m_channel->i2c, 0);
    unlock();
    return ret;
}
int I2CDriver::writeSlave(const char* data, int length) {
    m_modeSlave = true;
    lockNconfig();
    int ret = i2cRtos_slave_write(&m_channel->i2c, data, length);
    unlock();
    return ret;
}
int I2CDriver::writeSlave(int data) {
    m_modeSlave = true;
    lockNconfig();
    int ret = i2cRtos_byte_write(&m_channel->i2c, data);
    unlock();
    return ret;
}
