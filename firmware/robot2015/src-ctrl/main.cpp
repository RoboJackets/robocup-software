#include "robot.hpp"

// ** DON'T INCLUDE <iostream>! THINGS WILL BREAK! **
#include <cstdarg>
#include <ctime>
#include <string>
#include <array>

#include <logger.hpp>

#include "commands.hpp"
#include "io-expander.hpp"
#include "TaskSignals.hpp"
//#include "neostrip.hpp"
//#include "buzzer.hpp"

#define _EXTERN extern "C"

void Task_Controller(void const* args);

/**
 * @brief      { Sets the hardware configurations for the status LEDs & places
 * into the given state }
 *
 * @param[in]  state  { the next state of the LEDs }
 */
void statusLights(bool state) {
    DigitalInOut init_leds[] = {{RJ_BALL_LED, PIN_OUTPUT, OpenDrain, !state},
                                {RJ_RX_LED, PIN_OUTPUT, OpenDrain, !state},
                                {RJ_TX_LED, PIN_OUTPUT, OpenDrain, !state},
                                {RJ_RDY_LED, PIN_OUTPUT, OpenDrain, !state}};

    for (int i = 0; i < 4; i++) init_leds[i].mode(PullUp);
}

/**
 * @brief      { Turns all status LEDs on }
 *
 * @param      args  { nothing }
 */
void statusLightsON(void const* args) { statusLights(1); }

/**
 * @brief      { Turns all status LEDs on }
 *
 * @param      args  { nothing }
 */
void statusLightsOFF(void const* args) { statusLights(0); }

// void sampleInputs()
// {
//  // Set global variables here for the config input values from the IO
// Expander.
//  // This is where the robot's ID comes from, so it's pretty important.
//  LOG(SEVERE, "Interrupt triggered!");
// }

/**
 * [main Main The entry point of the system where each submodule's thread is
 * started.]
 * @return  [none]
 */
int main() {
    // NeoStrip rgbLED(RJ_NEOPIXEL, 1);
    // rgbLED.setBrightness(0.2);
    // rgbLED.setPixel(0, 0x00, 0xFF, 0x00);
    // rgbLED.write();

    // Turn on some startup LEDs to show they're working, they are turned off
    // before we hit the while loop
    statusLightsON(nullptr);

    // Set the default logging configurations
    isLogging = RJ_LOGGING_EN;
    rjLogLevel = INIT;

    /* Always send out an empty line at startup for keeping the console
     * clean on after a 'reboot' command is called;
     */
    if (isLogging) {
        // reset the console's default settings and enable the cursor
        printf("\033[0m\033[?25h");
        fflush(stdout);
    }

    // clear any extraneous rx serial bytes
    if (1) {
        Serial s(RJ_SERIAL_RXTX);
        while (s.readable()) {
            s.getc();
        }
        while (!s.writeable()) { /* wait */
        }
    }

    // Setup the interrupt priorities before launching each subsystem's task
    // thread.
    setISRPriorities();

    // Start a periodic blinking LED to show system activity
    DigitalOut ledOne(LED1, 0);
    RtosTimer live_light(imAlive, osTimerPeriodic, (void*)&ledOne);
    live_light.start(RJ_LIFELIGHT_TIMEOUT_MS);

    // TODO: write a function that will recalibrate the radio for this.
    // Reset the ticker on every received packet. For now, we just blink an LED.
    // DigitalOut led4(LED4, 0);
    // RtosTimer radio_timeout_task(imAlive, osTimerPeriodic, (void*)&led4);
    // radio_timeout_task.start(300);

    // Flip off the startup LEDs after a timeout period
    RtosTimer init_leds_off(statusLightsOFF, osTimerOnce);
    init_leds_off.start(RJ_STARTUP_LED_TIMEOUT_MS);

    // This is where the FPGA is actually configured with the bitfile's name
    // passed in
    bool fpga_ready = FPGA::Instance()->Init("/local/rj-fpga.nib");

    /* We MUST wait for the FPGA to COMPLETELY configure before moving on
     * because of
     * threading with the shared the SPI. If we don't explicitly halt main (it's
     * just
     * considered another thread), then the chip select lines will not be in the
     * correct
     * states for communicating between devices exclusively. Many bus faults
     * occured on the
     * road to finding this problem and then determining how to fix it.
     */
    if (fpga_ready == true) {
        LOG(INIT, "FPGA Configuration Successful!");
        osSignalSet(Thread::gettid(), MAIN_TASK_CONTINUE);
    } else {
        LOG(FATAL, "FPGA Configuration Failed!");
        osSignalSet(Thread::gettid(), MAIN_TASK_CONTINUE);
    }

    Thread::signal_wait(MAIN_TASK_CONTINUE, osWaitForever);

    // // Setup the IO Expander's hardware
    // MCP23017::Init();

    // // Setup some extended LEDs and turn them on
    // IOExpanderDigitalOut led_err_m1(IOExpanderPinB0);
    // led_err_m1 = 1;

    // uint8_t robot_id = MCP23017::digitalWordRead() & 0x0F;
    // LOG(INIT, "Robot ID:\t%u", robot_id);

    motors_Init();

    // Start the thread task for the on-board control loop
    Thread controller_task(Task_Controller, nullptr, osPriorityHigh);

    // Start the thread task for handling radio communications
    Thread comm_task(Task_CommCtrl, nullptr, osPriorityAboveNormal);

    // Start the thread task for the serial console
    Thread console_task(Task_SerialConsole, nullptr, osPriorityBelowNormal);

// Attach an interrupt callback for setting the buttons/switches states
// into the firmware anytime one of them changes

// InterruptIn configInputs(RJ_IOEXP_INT);
// configInputs.rise(&sampleInputs);

#if RJ_WATCHDOG_TIMER_EN
    // Enable the watchdog timer if it's set in configurations.
    Watchdog::Set(RJ_WATCHDOG_TIMER_VALUE);
#endif

    DigitalOut rdy_led(RJ_RDY_LED, !fpga_ready);

    if (fpga_ready) {
        // NeoStrip rgbLED(RJ_NEOPIXEL, 1);
        // rgbLED.setBrightness(1.0);
        // rgbLED.setPixel(0, 0x00, 0xFF, 0x00);
        // rgbLED.write();
    }

    // LOG(INIT, "FPGA git commit hash:\t0x%08X", FPGA::Instance()->git_hash());

    // Clear out the header for the console
    FPGA::Instance()->motors_en(true);

    while (true) {
        rdy_led = !fpga_ready;
        Thread::wait(
            2000);  // Ping back to main every now and then seems to perform
        // better than calling Thread::yeild() for some
        // reason?
    }
}

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
    away as the variables never actually get used.  If the debugger won't show
    the
    values of the variables, make them global my moving their declaration
    outside
    of this function. */
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
    };
}

_EXTERN void NMI_Handler() {
    printf("NMI Fault!\n");
    // NVIC_SystemReset();
}

_EXTERN void MemManage_Handler() {
    printf("MemManage Fault!\n");
    // NVIC_SystemReset();
}

_EXTERN void BusFault_Handler() {
    printf("BusFault Fault!\n");
    // NVIC_SystemReset();
}

_EXTERN void UsageFault_Handler() {
    printf("UsageFault Fault!\n");
    // NVIC_SystemReset();
}
