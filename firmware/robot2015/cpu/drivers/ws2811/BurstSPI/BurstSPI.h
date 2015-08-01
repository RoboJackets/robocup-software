#ifndef BURSTSPI_H
#define BURSTSPI_H

#include "mbed.h"


/** An SPI Master, used for communicating with SPI slave devices at very high speeds
 *
 * The default mbed SPI class allows for communication via the SPI bus at high clock frequencies,
 * however at these frequencies there is alot of overhead from the mbed code.
 * While this code makes sure your code is alot more robust, it is also relative slow.
 * This library adds to your default SPI commands some extra commands to transmit data rapidly with
 * very little overhead. Downsides are that currently it is TX only (all RX packets are discarded),
 * and it requires some extra commands.
 *
 * Example:
 * @code
 *  //Send 1000 SPI packets as fast as possible
 *  spi.setFormat();
 *  for (int i = 0; i<1000; i++)
 *    spi.fastWrite(data[i]);
 *  spi.clearRX();
 * @endcode
 *
 * As an example, writing 76,800 16-bit data packets to an LCD screen at 48MHz requires 111ms with
 * the normal mbed library. With this library it takes 25ms, which is also the theoretical
 * amount of time it should take. If you are running at 1MHz this will do alot less.
 */
class BurstSPI : public SPI
{
public:
    /** Create a SPI master connected to the specified pins
    *
    * Pin Options:
    *  (5, 6, 7) or (11, 12, 13)
    *
    *  mosi or miso can be specfied as NC if not used
    *
    *  @param mosi SPI Master Out, Slave In pin
    *  @param miso SPI Master In, Slave Out pin
    *  @param sclk SPI Clock pin
    */
    BurstSPI(PinName mosi, PinName miso, PinName sclk) : SPI(mosi, miso, sclk) {};

    /** Put data packet in the SPI TX FIFO buffer
    *
    *  If there is no space in the FIFO buffer it will block until there is space.
    * The FIFO buffer will automatically send the packets. There is no receiving here, only transmitting.
    *
    *  @param data Data to be sent to the SPI slave
    */
    void fastWrite(int data);

    /** Use this function before fastWrite to set the correct settings
    *
    * It is not needed to use this if the last SPI commands were either normal SPI transmissions,
    * or setting different format/frequency for this object. It is required to call this
    * function when several SPI objects use the same peripheral, and your last transmission was
    * from a different object with different settings. Not sure if you should use it?
    * Use it, it takes very little time to execute, so can't hurt.
    */
    void setFormat( void ) {
        format(_bits, _mode);
        frequency(_hz);
    }

    /** After you are done with fastWrite, call this function
    *
    * FastWrite simply fills the SPI's (SSP's actually) TX FIFO buffer as fast as it can,
    * and that is the only thing it does. It doesn't do anything with received packages (currently, may change),
    * so the the RX buffer is full with unneeded packets. This function waits until transmission is finished,
    * and clears the RX buffer. You always have to call this before you want to receive
    * SPI data after using fastWrite.
    */
    void clearRX( void );


    //Just for documentation:
#if 0
    /** Configure the data transmission format
     *
     *  @param bits Number of bits per SPI frame (4 - 16)
     *  @param mode Clock polarity and phase mode (0 - 3)
     *
     * @code
     * mode | POL PHA
     * -----+--------
     *   0  |  0   0
     *   1  |  0   1
     *   2  |  1   0
     *   3  |  1   1
     * @endcode
     */
    void format(int bits, int mode = 0);

    /** Set the spi bus clock frequency
     *
     *  @param hz SCLK frequency in hz (default = 1MHz)
     */
    void frequency(int hz = 1000000);

    /** Write to the SPI Slave and return the response
     *
     *  @param value Data to be sent to the SPI slave
     *
     *  @returns
     *    Response from the SPI slave
    */
    virtual int write(int value);
#endif

};

#endif