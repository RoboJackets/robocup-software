#pragma once

#include "RTP.hpp"

#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif

typedef enum {
    RESET = 0,
    SET = !RESET
} FlagStatus, ITStatus;

typedef enum {
    DISABLE = 0,
    ENABLE = !DISABLE
} FunctionalState;

typedef uint32_t ERR_t;
