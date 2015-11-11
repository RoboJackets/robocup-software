#pragma once

// This is where the signals for different threads are defined for use across mutliple files

#define MAIN_TASK_CONTINUE                  (0x01)
#define CONSOLE_TASK_START_SIGNAL           (0x01)
#define COMMUNICATION_TASK_START_SIGNAL     (CONSOLE_TASK_START_SIGNAL)
