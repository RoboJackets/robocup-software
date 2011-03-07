#pragma once

#define DEFAULT_CONTROLLER 0

// If not null, this function is called every update cycle to generate new motor commands.
extern void (*controller)(void);
