#pragma once

// This is where the signals for different threads are defined for use across
// mutliple files

static const uint32_t MAIN_TASK_CONTINUE = 1 << 0;
static const uint32_t SUB_TASK_CONTINUE = 1 << 1;
