# Firmware {#t2016firmware}


# Initialization

-   Initialize Kicker(attiny) and FPGA using binary files stored on MBED
    -   Compilation for these files is separate from firmware MBED runs
-   Initialize any other devices (radio, MPU, etc.)
-   Start threads


# RTOS (Threads)

-   Threads allow us to run multiple segments of code in "parallel"
-   MBED only has one core so only one thread executed at once
-   RTOS is used to control the running of these threads
-   Threads can be running, ready (in queue to be run), or waiting (for some event like a timer or interrupt)

![img](//developer.mbed.org/media/uploads/emilmont/xthreadstatus.png.pagespeed.ic.c21fE5uss-.jpg)


## Main

-   Watchdog (reset MBED if it doesn't go back to main in certain time)
-   Control LEDs (neopixel & LEDs connected to IO Expander)
-   Respond to buttons/switches(rotary selector & dip switch)


## Other threads

-   Console
    -   Read and write to serial for the console
    -   Commands for console defined in commands.cpp
-   Controller
    -   Send duty cycles to FPGA to control motor speeds
-   Radio
    -   Two RX threads, one TX thread (see Radio section)


# Radio

-   How commands are sent to robot (motor speeds, kick, etc.)
-   How information is sent back to the base station (various robot statuses)


## CommLink

-   Used as hardware abstraction layer to communicate from higher level firmware to radio hardware (through drivers)
-   What we use to call functions to get/send data through radio
-   Other radio drivers (cc1201 and decawave) are derived from this in order to implement the specific hardware communication
-   Runs RX thread waiting for radio interrupts


## CommModule

-   Used to handle packets being sent and received with radio
-   RX/TX queues (as threads) waiting for other threads to put packets in the queue that are ready to be sent or have been received and need to be processed


# Shared SPI

-   Allows MBED (master) to communicate with multiple devices through same pins
-   Devices (slaves) currently used are radio, kicker (ATtiny), and FPGA
-   This ensures multiple threads aren't trying to communicate on these shared lines all at once


# Hardfault Handler

-   Written in assembly to call function when the mbed hard faults (crashes)
-   Used to print out information to console useful for debugging


# FPGA (verilog)

-   FPGA is programming logic gates which allows for many parallel operations
-   Reads in hall sensors for motors
    -   Used to determine which phase to power to keeps motors running
-   Gets encoder values for how fast motor is spinning
    -   Given number of encoder steps passed in period of time
    -   This value can be read by MBED over SPI

-   Powers correct phases for motors (by sending signals to motor drivers)
    -   Powers the next phase after the current phase given by hall sensors
    -   Powers the motors using the duty cycle sent from the MBED
        -   Adjusting the duty cycle allows us to control motor speeds
-   Acts as SPI slave to receive motor speeds from MBED and has second SPI communication where it acts as SPI master to communicate with motor drivers
-   Pin configurations found in robocup.ucf


# Base Station

-   MBED with only radio connected
-   Uses MBED serial lines to communicate with computer running soccer
-   Commands are sent from soccer to base station then to each robot
-   Base station recieves replies back from robots then sends this to soccer
