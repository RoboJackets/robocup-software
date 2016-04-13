// ** DON'T INCLUDE <iostream>! THINGS WILL BREAK! **
#include <ctime>
#include <string>
#include <array>

#include <rtos.h>

#include <helper-funcs.hpp>
#include <watchdog.hpp>
#include <logger.hpp>
#include <assert.hpp>

#include "robot-devices.hpp"
#include "task-signals.hpp"
#include "commands.hpp"
#include "fpga.hpp"
#include "io-expander.hpp"
#include "neostrip.hpp"
#include "CC1201.cpp"
#include "BallSense.hpp"
#include "SharedSPI.hpp"
#include "KickerBoard.hpp"
#include "RadioProtocol2011.hpp"
#include "RtosTimerHelper.hpp"
#include "io-expander.hpp"
#include "RotarySelector.hpp"

using namespace std;

void Task_Controller(void const* args);

/**
 * @brief Sets the hardware configurations for the status LEDs & places
 * into the given state
 *
 * @param[in] state The next state of the LEDs
 */
void statusLights(bool state) {
    DigitalOut init_leds[] = {
        {RJ_BALL_LED}, {RJ_RX_LED}, {RJ_TX_LED}, {RJ_RDY_LED}};
    // the state is inverted because the leds are wired active-low
    for (DigitalOut& led : init_leds) led = !state;
}

/**
 * The entry point of the system where each submodule's thread is started.
 */
int main() {
    // Store the thread's ID
    const osThreadId mainID = Thread::gettid();
    ASSERT(mainID != nullptr);

    // clear any extraneous rx serial bytes
    Serial s(RJ_SERIAL_RXTX);
    while (s.readable()) s.getc();

    // set baud rate to higher value than the default for faster terminal
    s.baud(57600);

    // Turn on some startup LEDs to show they're working, they are turned off
    // before we hit the while loop
    statusLights(true);

    // Set the default logging configurations
    isLogging = RJ_LOGGING_EN;
    rjLogLevel = INIT;

    /* Always send out an empty line at startup for keeping the console
     * clean on after a 'reboot' command is called;
     */
    if (isLogging) {
        // reset the console's default settings and enable the cursor
        printf("\033[m");
        fflush(stdout);
    }

    // Setup the interrupt priorities before launching each subsystem's task
    // thread.
    setISRPriorities();

    // Initialize and start ball sensor
    BallSense ballSense(RJ_BALL_EMIT, RJ_BALL_DETECTOR);
    ballSense.start(100);  // TODO(justin): choose smarter update frequency
    DigitalOut ballSenseStatusLED(RJ_BALL_LED, 1);

    // Force off since the neopixel's hardware is stateless from previous
    // settings
    NeoStrip rgbLED(RJ_NEOPIXEL, 2);
    rgbLED.clear();

    // Set the RGB LEDs to a medium blue while the threads are started up
    float defaultBrightness = 0.02f;
    rgbLED.brightness(3 * defaultBrightness);
    rgbLED.setPixel(0, NeoColorBlue);
    rgbLED.setPixel(1, NeoColorBlue);
    rgbLED.write();

    // Start a periodic blinking LED to show system activity
    // This is set to never timeout, so it will only stop if the system halts
    StrobingTimeoutLEDs<4> liveLight({LED1, LED2, LED3, LED4},
                                     RJ_LIFELIGHT_TIMEOUT_MS, osWaitForever);

    // Flip off the startup LEDs after a timeout period
    RtosTimerHelper init_leds_off([]() { statusLights(false); }, osTimerOnce);
    init_leds_off.start(RJ_STARTUP_LED_TIMEOUT_MS);

    /// A shared spi bus used for the fpga and cc1201 radio
    shared_ptr<SharedSPI> sharedSPI =
        make_shared<SharedSPI>(RJ_SPI_MOSI, RJ_SPI_MISO, RJ_SPI_SCK);
    sharedSPI->format(8, 0);  // 8 bits per transfer

    // Initialize and configure the fpga with the given bitfile
    FPGA::Instance = new FPGA(sharedSPI, RJ_FPGA_nCS, RJ_FPGA_INIT_B,
                              RJ_FPGA_PROG_B, RJ_FPGA_DONE);
    bool fpga_ready = FPGA::Instance->configure("/local/rj-fpga.nib");

    if (fpga_ready) {
        LOG(INIT, "FPGA Configuration Successful!");
    } else {
        LOG(FATAL, "FPGA Configuration Failed!");
    }

    DigitalOut rdy_led(RJ_RDY_LED, !fpga_ready);

    // Initialize kicker board
    // TODO: clarify between kicker nCs and nReset
    KickerBoard kickerBoard(sharedSPI, RJ_KICKER_nCS, RJ_KICKER_nRESET,
                            "/local/rj-kickr.nib");
    bool kickerReady = kickerBoard.flash(true, true);

    // Init IO Expander and turn all LEDs on.  The first parameter to config()
    // sets the first 8 lines to input and the last 8 to output.  The pullup
    // resistors and polarity swap are enabled for the 4 rotary selector lines.
    MCP23017 ioExpander(RJ_I2C_SDA, RJ_I2C_SCL, RJ_IO_EXPANDER_I2C_ADDRESS);
    ioExpander.config(0x00FF, 0x00f0, 0x00f0);
    ioExpander.writeMask((uint16_t)~IOExpanderErrorLEDMask,
                         IOExpanderErrorLEDMask);

    // rotary selector for shell id
    RotarySelector<IOExpanderDigitalInOut> rotarySelector(
        {IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT0,
                                MCP23017::DIR_INPUT),
         IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT1,
                                MCP23017::DIR_INPUT),
         IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT2,
                                MCP23017::DIR_INPUT),
         IOExpanderDigitalInOut(&ioExpander, RJ_HEX_SWITCH_BIT3,
                                MCP23017::DIR_INPUT)});

    // Startup the 3 separate threads, being sure that we wait for it
    // to signal back to us that we can startup the next thread. Not doing
    // so results in weird wierd things that are really hard to debug. Even
    // though this is multi-threaded code, that dosen't mean it's
    // a multi-core system.

    // Start the thread task for the on-board control loop
    Thread controller_task(Task_Controller, mainID, osPriorityHigh,
                           DEFAULT_STACK_SIZE / 2);
    Thread::signal_wait(MAIN_TASK_CONTINUE, osWaitForever);

    // Start the thread task for the serial console
    Thread console_task(Task_SerialConsole, mainID, osPriorityBelowNormal);
    Thread::signal_wait(MAIN_TASK_CONTINUE, osWaitForever);

    // Initialize the CommModule and CC1201 radio
    InitializeCommModule(sharedSPI);

    // Make sure all of the motors are enabled
    motors_Init();

    RadioProtocol2011 radioProtocol(CommModule::Instance, global_radio);
    radioProtocol.start();
    radioProtocol.setUID(2);  // TODO: remove
    radioProtocol.rxCallback = [](uint8_t* msg) {
        // TODO: parse @msg

        // TODO: create actual message
        // return vector<uint8_t>({
        //     // uid,
        //     // last_rssi,
        //     // battery_level,
        //     // kicker_status, // TODO: kicker failure?
        //     // motor_status,
        //     // failures, // bit field of failures, status, etc
        //     // kicker_voltage
        // });
        LOG(INIT, "rx!");
        return vector<uint8_t>(10, 1);
    };

    // Set the watdog timer's initial config
    Watchdog::Set(RJ_WATCHDOG_TIMER_VALUE);

    // Release each thread into its operations in a structured manner
    controller_task.signal_set(SUB_TASK_CONTINUE);
    console_task.signal_set(SUB_TASK_CONTINUE);

    osStatus tState = osThreadSetPriority(mainID, osPriorityNormal);
    ASSERT(tState == osOK);

    unsigned int ll = 0;

    while (true) {
        // make sure we can always reach back to main by
        // renewing the watchdog timer periodicly
        Watchdog::Renew();

        // periodically reset the console text's format
        ll++;
        if ((ll % 4) == 0) {
            printf("\033[m");
            fflush(stdout);
        }

        Thread::wait(RJ_WATCHDOG_TIMER_VALUE * 250);

        // the value is inverted because this led is wired active-low
        ballSenseStatusLED = !ballSense.have_ball();

        // Pack errors into bitmask
        uint16_t errorBitmask = !global_radio->isConnected()
                                << RJ_ERR_LED_RADIO;

        // add motor errors to bitmask
        static const auto motorErrLedMapping = {
            make_pair(0, RJ_ERR_LED_M1), make_pair(1, RJ_ERR_LED_M2),
            make_pair(2, RJ_ERR_LED_M3), make_pair(3, RJ_ERR_LED_M4),
            make_pair(4, RJ_ERR_LED_DRIB)};
        for (auto& pair : motorErrLedMapping) {
            const motorErr_t& status = global_motors[pair.first].status;
            errorBitmask |= !status.hallOK << pair.second;
        }

        // Set error-indicating leds on the control board
        ioExpander.writeMask(~errorBitmask, IOExpanderErrorLEDMask);

        // Set error indicators
        if (!fpga_ready) {
            // orange - error
            rgbLED.brightness(4 * defaultBrightness);
            rgbLED.setPixel(0, NeoColorOrange);
        } else {
            // green - no error...yet
            rgbLED.brightness(defaultBrightness);
            rgbLED.setPixel(0, NeoColorGreen);
        }

        if (errorBitmask & RJ_ERR_LED_RADIO) {
            // orange - error
            rgbLED.brightness(6 * defaultBrightness);
            rgbLED.setPixel(1, NeoColorOrange);
        } else {
            // green - no error...yet
            rgbLED.brightness(6 * defaultBrightness);
            rgbLED.setPixel(1, NeoColorGreen);
        }

        if ((errorBitmask & RJ_ERR_LED_RADIO) && !fpga_ready) {
            // bright as hell to make sure they know
            rgbLED.brightness(10 * defaultBrightness);
            // well, damn. everything is broke as hell
            rgbLED.setPixel(0, NeoColorRed);
            rgbLED.setPixel(1, NeoColorRed);
        }
    }
}

#define _EXTERN extern "C"

_EXTERN void HardFault_Handler() {
    __asm volatile(
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, hard_fault_handler_2_const                        \n"
        " bx r2                                                     \n"
        " hard_fault_handler_2_const: .word HARD_FAULT_HANDLER    	\n");
}

_EXTERN void HARD_FAULT_HANDLER(uint32_t* stackAddr) {
    /* These are volatile to try and prevent the compiler/linker optimising them
     * away as the variables never actually get used.  If the debugger won't
     * show the values of the variables, make them global my moving their
     * declaration outside of this function. */
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t r2;
    volatile uint32_t r3;
    volatile uint32_t r12;
    volatile uint32_t lr;  /* Link register. */
    volatile uint32_t pc;  /* Program counter. */
    volatile uint32_t psr; /* Program status register. */

    r0 = stackAddr[0];
    r1 = stackAddr[1];
    r2 = stackAddr[2];
    r3 = stackAddr[3];
    r12 = stackAddr[4];
    lr = stackAddr[5];
    pc = stackAddr[6];
    psr = stackAddr[7];

    LOG(FATAL,
        "\r\n"
        "================================\r\n"
        "========== HARD FAULT ==========\r\n"
        "\r\n"
        "  MSP:\t0x%08X\r\n"
        "  HFSR:\t0x%08X\r\n"
        "  CFSR:\t0x%08X\r\n"
        "\r\n"
        "  r0:\t0x%08X\r\n"
        "  r1:\t0x%08X\r\n"
        "  r2:\t0x%08X\r\n"
        "  r3:\t0x%08X\r\n"
        "  r12:\t0x%08X\r\n"
        "  lr:\t0x%08X\r\n"
        "  pc:\t0x%08X\r\n"
        "  psr:\t0x%08X\r\n"
        "\r\n"
        "========== HARD FAULT ==========\r\n"
        "================================",
        __get_MSP, SCB->HFSR, SCB->CFSR, r0, r1, r2, r3, r12, lr, pc, psr);

    // do nothing so everything remains unchanged for debugging
    while (true) {
    }
}

_EXTERN void NMI_Handler() { std::printf("NMI Fault!\n"); }

_EXTERN void MemManage_Handler() { std::printf("MemManage Fault!\n"); }

_EXTERN void BusFault_Handler() { std::printf("BusFault Fault!\n"); }

_EXTERN void UsageFault_Handler() { std::printf("UsageFault Fault!\n"); }
