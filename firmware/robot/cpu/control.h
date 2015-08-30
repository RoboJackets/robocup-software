#pragma once

typedef struct {
    // Name for the "run" command.
    const char* name;

    // Any of the pointers may be null.

    // Called before the controller is first used
    void (*init)(int argc, const char* argv[]);

    // Called when a new forward packet is received
    void (*received)(void);

    // Called just after sensors are updated and just before commands are sent
    // to the FPGA.
    // Put the speed controller here.
    void (*update)(void);
} controller_info_t;

extern const controller_info_t* default_controller;
extern const controller_info_t controllers[];

// If not null, this function is called every update cycle to generate new motor
// commands.
extern const controller_info_t* controller;
