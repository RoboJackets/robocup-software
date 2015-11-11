#include "i2cRtos_api.hpp"

#if DEVICE_I2C

#include <mbed.h>
#include <rtos.h>
// #include "cmsis_os.h"

#include "logger.hpp"

// little helpers cloned from official i2c api
#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
#define I2C_CONSET(x)       (x->i2c->I2CONSET)
#define I2C_CONCLR(x)       (x->i2c->I2CONCLR)
#define I2C_STAT(x)         (x->i2c->I2STAT)
#define I2C_DAT(x)          (x->i2c->I2DAT)
#elif defined(TARGET_LPC11U24)
#define I2C_CONSET(x)       (x->i2c->CONSET)
#define I2C_CONCLR(x)       (x->i2c->CONCLR)
#define I2C_STAT(x)         (x->i2c->STAT)
#define I2C_DAT(x)          (x->i2c->DAT)
#endif

// isr/thread data transfer struct
enum I2cIsrCmd {
    readMst,
    writeMst,
    readSlv,
    writeSlv,
    waitSI
};

struct I2cIsrTransfer {
    i2c_t* obj;
    enum I2cIsrCmd cmd;
    int len;
    int cnt;
    int stat;
    char* rData;
    const char* wData;
};
// one for each channel
// "volatile" has been omitted since ISR and thread do not run simultaneously, hopefully ok
static struct I2cIsrTransfer i2c_transfer[2];

// struct holding IRQ and semaphore ID
// needed for thread<->isr communication/activation
struct IsrIrqSem {
    IRQn_Type irq;
    osSemaphoreId sem;
};

static struct IsrIrqSem isrIrqSem[2]; // one for each channel


// little helpers cloned from official i2c api
static inline void i2c_conclr(i2c_t *obj, int start, int stop, int interrupt, int acknowledge)
{
    I2C_CONCLR(obj) = (start << 5)
                      | (stop << 4)
                      | (interrupt << 3)
                      | (acknowledge << 2);
}
static inline void i2c_conset(i2c_t *obj, int start, int stop, int interrupt, int acknowledge)
{
    I2C_CONSET(obj) = (start << 5)
                      | (stop << 4)
                      | (interrupt << 3)
                      | (acknowledge << 2);
}
static inline void i2c_clear_SI(i2c_t *obj)
{
    i2c_conclr(obj, 0, 0, 1, 0);
}
static inline int i2c_status(i2c_t *obj)
{
    return I2C_STAT(obj);
}

// ISR routines
// implements the same read/write sequences as the official i2c lib
static void i2cRtos_isr(uint32_t ch)
{
    struct I2cIsrTransfer* tr = &(i2c_transfer[ch]);
    if (tr->cmd == waitSI) {
        // just waiting for an interrupt after a byte read/write or slave receive call
        osSemaphoreRelease(isrIrqSem[ch].sem);
        NVIC_DisableIRQ(isrIrqSem[ch].irq);
        return;
    }

    int stat = i2c_status(tr->obj);
    int stay = 0;

    switch (tr->cmd) {

    case readMst:
        switch (stat) {

        case 0x50: // Data byte has been received; ACK has been returned.
            (tr->rData)[tr->cnt] = (char)(I2C_DAT(tr->obj) & 0xff);

        case 0x40: // SLA+R has been transmitted; ACK has been received.
            ++(tr->cnt);
            if (tr->cnt != tr->len - 1)
                i2c_conset(tr->obj, 0, 0, 0, 1);
            else
                i2c_conclr(tr->obj, 0, 0, 0, 1); // do not ack the last byte read
            stay = 1;
            break;

        case 0x58: // Data byte has been received; NOT ACK has been returned.
            (tr->rData)[tr->cnt] = (char)(I2C_DAT(tr->obj) & 0xff);
            stat = 0;
            break;
        }
        break;

    case writeMst:
        switch (stat) {

        case 0x18: // SLA+W has been transmitted; ACK has been received.

        case 0x28: // SLA+W has been transmitted; NOT ACK has been received.
            if (++(tr->cnt) != tr->len) {
                I2C_DAT(tr->obj) = (tr->wData)[tr->cnt];
                stay = 1;
            } else {
                stat = 0;
            }
        }
        break;

    case readSlv:
        ++(tr->cnt);
        if (stat == 0x80 || stat == 0x90) // Previously addressed with own SLA address(0x80) or geberal call (0x90); DATA has been received; ACK has been returned.
            (tr->rData)[tr->cnt] = I2C_DAT(tr->obj) & 0xFF;
        stay = (stat == 0x80 || stat == 0x90 || stat == 0x060 || stat == 0x70) && (tr->cnt < tr->len - 1);
        // 60: Own SLA+W has been received; ACK has been returned. ... SLV+W???
        // 70: General Call address (0x00) has been received; ACK has been returned.
        break;

    case writeSlv:
        ++(tr->cnt);
        stay = tr->cnt < tr->len && stat == 0xb8; // Data byte in I2DAT has been transmitted; ACK has been received.
        if (stay) I2C_DAT(tr->obj) = tr->wData[tr->cnt];
        break;
    }

    if (stay) {
        // sequence not finished => stay in ISR mode and trigger next i2c by clearing he SI bit
        i2c_clear_SI(tr->obj);
    } else {
        // sequence finished or unexpected state has been reported
        // => bail out of isr mode and return last status received
        tr->stat = stat;
        osSemaphoreRelease(isrIrqSem[ch].sem);
        NVIC_DisableIRQ(isrIrqSem[ch].irq);
    }
}


#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
static void i2cRtos_isr_ch0()
{
    i2cRtos_isr(0);
}
#endif


static void i2cRtos_isr_ch1()
{
    i2cRtos_isr(1);
}


// determine channel
static inline uint32_t i2c_get_channel(const i2c_t *obj)
{

#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
    switch ((unsigned int)obj->i2c) {
    case I2C_1:
        return 0;
    case I2C_2:
        return 1;
    default:
        LOG(SEVERE, "Dial911 i2c_get_channel: Invaid I2CName");
    }
#endif

    return 1;
}


// Enable IRQ and wait for ISR sequence to be finished
static inline void i2cRtos_wait_and_see(i2c_t *obj, int ch, uint32_t tmOut)  //evillive
{
    struct IsrIrqSem* iis = &(isrIrqSem[ch]);
    __disable_irq(); // evil, but don't want the next three lines to be interrupted
    i2c_clear_SI(obj);
    NVIC_ClearPendingIRQ(iis->irq);
    NVIC_EnableIRQ(iis->irq);
    __enable_irq();
    if (osSemaphoreWait(iis->sem, tmOut) != 1) NVIC_DisableIRQ(iis->irq); // time out => diable the IRQ
}

// just wait for a generic i2c interrupt
static inline void i2cRtos_waitSI(i2c_t *obj, uint32_t tmOut)
{
    int ch = i2c_get_channel(obj);
    i2c_transfer[ch].cmd = waitSI;
    i2cRtos_wait_and_see(obj, ch, tmOut);
}

// master mode read sequence
int i2cRtos_read(i2c_t *obj, int address, char *data, int length, int stop)
{
    //gpio_write(&gpio[1], 1);
    int stat = i2c_start(obj);
    if ((stat != 0x10) && (stat != 0x08)) {
        i2cRtos_stop(obj); // use freeze free stop if the start has failed
        return stat;
    }
    //gpio_write(&gpio[1], 0);
    int ch = i2c_get_channel(obj);
    struct I2cIsrTransfer* tr = &(i2c_transfer[ch]);
    tr->obj = obj;
    tr->cmd = readMst;
    tr->len = length;
    tr->cnt = -1;
    tr->rData = data;
    I2C_DAT(obj) = address | 0x01;  // initiate address+R write and enter isr mode
    i2cRtos_wait_and_see(obj, ch, 2 + (length >> 2)); // timeout (2+len/4)ms
    stat = tr->stat;
    //gpio_write(&gpio[1], 1);
    if (stat || stop) i2cRtos_stop(obj);
    //gpio_write(&gpio[1], 0);
    return stat;
}

// master mode write sequence
int i2cRtos_write(i2c_t *obj, int address, const char *data, int length, int stop)
{
    //gpio_write(&gpio[1], 1);
    int status = i2c_start(obj);
    if ((status != 0x10) && (status != 0x08)) {
        i2cRtos_stop(obj); // use freeze free stop if the start has failed
        return status;
    }
    //gpio_write(&gpio[1], 0);
    int ch = i2c_get_channel(obj);
    struct I2cIsrTransfer* tr = &(i2c_transfer[ch]); // evilive fill it locally and then copy it in one go to (volatile) mem?
    tr->obj = obj;
    tr->cmd = writeMst;
    tr->len = length;
    tr->cnt = -1;
    tr->wData = data;
    I2C_DAT(obj) = address & 0xfe; // initiate address+W write and enter isr mode
    i2cRtos_wait_and_see(obj, ch, 2 + (length >> 2)); // timeout (2+len/4)ms
    //i2c_clear_SI(obj); // ... why? Also in official lib ... I guess this is the "write instead of start" bug
    status = tr->stat;
    //gpio_write(&gpio[1], 1);
    if (status || stop) i2cRtos_stop(obj);
    //gpio_write(&gpio[1], 0);
    return status;
}

// read single byte from bus (master/slave)
int i2cRtos_byte_read(i2c_t *obj, int last)
{
    if (last) {
        i2c_conclr(obj, 0, 0, 0, 1); // send a NOT ACK
    } else {
        i2c_conset(obj, 0, 0, 0, 1); // send a ACK
    }
    i2cRtos_waitSI(obj, 2);
    return (I2C_DAT(obj) & 0xff);
}

// write single byte to bus (master/slave)
int i2cRtos_byte_write(i2c_t *obj, int data)
{
    I2C_DAT(obj) = (data & 0xff);
    i2cRtos_waitSI(obj, 2);
    int stat = i2c_status(obj);
    return (stat == 0x18 || stat == 0x28 || stat == 0x40 || stat == 0xb8);
}

// freeze free i2c stop
// the busy wait without timeout of the official i2c lib
// might freeze the mbed if someone on the bus keeps the clock down
// and prevents LPC's i2c controller to create a stop condition
int i2cRtos_stop(i2c_t *obj)
{
    i2c_conset(obj, 0, 1, 0, 0);
    i2c_clear_SI(obj);
    uint32_t t0 = us_ticker_read();
    uint32_t dt = 0;
    while ((I2C_CONSET(obj) & (1 << 4)) && dt < 23) {
        dt = us_ticker_read() - t0;
    }
    return dt < 23;
}


#if DEVICE_I2CSLAVE

// determine slave's receive mode. Blocking with timeout
int i2cRtos_slave_receive(i2c_t *obj, uint32_t tmOut)
{
    int retval = i2c_slave_receive(obj);
    //check for pending requests
    if (retval)return retval; // there is one => bail out
    // No request? Wait for it! ... with time out
    i2cRtos_waitSI(obj, tmOut);
    // check again for pending requests
    return i2c_slave_receive(obj);
}

// slave mode read sequence
int i2cRtos_slave_read(i2c_t *obj, char *data, int length)
{
    int ch = i2c_get_channel(obj);
    struct I2cIsrTransfer* tr = &(i2c_transfer[ch]); // evilive fill it locally and then copy it in one go to (volatile) mem?
    tr->obj = obj;
    tr->cmd = readSlv;
    tr->len = length;
    tr->cnt = -1;
    tr->rData = data;
    i2cRtos_wait_and_see(obj, ch, 2 + (length >> 2)); // timeout (1+len/4)ms
    if (tr->stat != 0xa0) {
        i2cRtos_stop(obj);
    }
    i2c_clear_SI(obj); // stop keeping scl low
    return tr->cnt;    // same weird return as in official lib
}

// slave mode write sequence
int i2cRtos_slave_write(i2c_t *obj, const char *data, int length)
{
    if (length <= 0) {
        return (0);
    }
    int ch = i2c_get_channel(obj);
    struct I2cIsrTransfer* tr = &(i2c_transfer[ch]); // evilive fill it locally and then copy it in one go to (volatile) mem?
    tr->obj = obj;
    tr->cmd = writeSlv;
    tr->len = length;
    tr->cnt = 0;
    tr->wData = data;
    I2C_DAT(obj) = data[0];
    i2cRtos_wait_and_see(obj, ch, 2 + (length >> 2)); // timeout (1+len/4)ms
    int status = tr->stat;
    if (status != 0xC0 && status != 0xC8) {
        i2cRtos_stop(obj);
    }
    i2c_clear_SI(obj); // stops keeping scl low
    return tr->cnt;
}
#endif

// setup semaphores and hook in ISRs
void i2cRtos_init(i2c_t *obj, PinName sda, PinName scl)
{
    i2c_init(obj, sda, scl);
    uint32_t ch = i2c_get_channel(obj);

#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
    static osSemaphoreDef(i2cIsrDrvSem_ch0);
    static osSemaphoreDef(i2cIsrDrvSem_ch1);
    switch (ch) {
    case 0:
        isrIrqSem[ch].irq = I2C1_IRQn;
        NVIC_SetVector(I2C1_IRQn, (uint32_t)i2cRtos_isr_ch0);
        isrIrqSem[ch].sem = osSemaphoreCreate(osSemaphore(i2cIsrDrvSem_ch0), 1);
        break;
    case 1:
        isrIrqSem[ch].irq = I2C2_IRQn;
        NVIC_SetVector(I2C2_IRQn, (uint32_t)i2cRtos_isr_ch1);
        isrIrqSem[ch].sem = osSemaphoreCreate(osSemaphore(i2cIsrDrvSem_ch1), 1);
        break;
    }
    osSemaphoreWait(isrIrqSem[ch].sem, osWaitForever);

#elif defined(TARGET_LPC11U24)
    static osSemaphoreDef(i2cIsrDrvSem_ch1);
    isrIrqSem[ch].irq = I2C_IRQn;
    NVIC_SetVector(I2C_IRQn, (uint32_t)i2cRtos_isr_ch1);
    isrIrqSem[ch].sem = osSemaphoreCreate(osSemaphore(i2cIsrDrvSem_ch1), 1);
    osSemaphoreWait(isrIrqSem[ch].sem, osWaitForever);
#else
#error warning "i2cRtos_init: Unsupported target"
#endif

}

#endif
