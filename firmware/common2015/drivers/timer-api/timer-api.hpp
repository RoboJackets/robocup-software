/*
 *  Timer API functions based on NXP software libraries
 */

#pragma once

#ifndef TARGET_LPC1768
#pragma message "Untested support for selected target!"
#endif

#include "cmsis.h"
#include "pinmap.h"

/* _BIT(n) sets the bit at position "n"
* _BIT(n) is intended to be used in "OR" and "AND" expressions:
* e.g., "(_BIT(3) | _BIT(7))".
*/
#undef _BIT
/* Set bit macro */
#define _BIT(n) (((unsigned int)(1)) << (n))

/** Macro to clear interrupt pending */
#define TIMER_IR_CLR(n) _BIT(n)
/** Macro for getting a timer match interrupt bit */
#define TIMER_MATCH_INT(n) (_BIT((n)&0x0F))
/** Macro for getting a capture event interrupt bit */
#define TIMER_CAP_INT(n) (_BIT((((n)&0x0F) + 4)))
/** Timer/counter enable bit */
#define TIMER_ENABLE ((uint32_t)(1 << 0))
/** Timer/counter reset bit */
#define TIMER_RESET ((uint32_t)(1 << 1))
/** Bit location for interrupt on MRx match, n = 0 to 3 */
#define TIMER_INT_ON_MATCH(n) (_BIT(((n)*3)))
/** Bit location for reset on MRx match, n = 0 to 3 */
#define TIMER_RESET_ON_MATCH(n) (_BIT((((n)*3) + 1)))
/** Bit location for stop on MRx match, n = 0 to 3 */
#define TIMER_STOP_ON_MATCH(n) (_BIT((((n)*3) + 2)))
/** Bit location for CAP.n on CRx rising edge, n = 0 to 3 */
#define TIMER_CAP_RISING(n) (_BIT(((n)*3)))
/** Bit location for CAP.n on CRx falling edge, n = 0 to 3 */
#define TIMER_CAP_FALLING(n) (_BIT((((n)*3) + 1)))
/** Bit location for CAP.n on CRx interrupt enable, n = 0 to 3 */
#define TIMER_INT_ON_CAP(n) (_BIT((((n)*3) + 2)))

/**
 * @brief Standard timer initial match pin state and change state
 */
enum TIMER_PIN_MATCH_STATE {
    TIMER_EXTMATCH_DO_NOTHING =
        0, /*!< Timer match state does nothing on match pin */
    TIMER_EXTMATCH_CLEAR = 1, /*!< Timer match state sets match pin low */
    TIMER_EXTMATCH_SET = 2,   /*!< Timer match state sets match pin high */
    TIMER_EXTMATCH_TOGGLE = 3 /*!< Timer match state toggles match pin */
};

/**
 * @brief Standard timer clock and edge for count source
 */
enum TIMER_CAP_SRC_STATE {
    TIMER_CAPSRC_RISING_PCLK = 0,  /*!< Timer ticks on PCLK rising edge */
    TIMER_CAPSRC_RISING_CAPN = 1,  /*!< Timer ticks on CAPn.x rising edge */
    TIMER_CAPSRC_FALLING_CAPN = 2, /*!< Timer ticks on CAPn.x falling edge */
    TIMER_CAPSRC_BOTH_CAPN = 3     /*!< Timer ticks on CAPn.x both edges */
};

static const PinMap PinMap_Timer[] = {{P0_6, LPC_TIM2_BASE, 3},
                                      {P0_7, LPC_TIM2_BASE, 3},
                                      {P0_8, LPC_TIM2_BASE, 3},
                                      {P0_9, LPC_TIM2_BASE, 3},
                                      {P0_10, LPC_TIM3_BASE, 3},
                                      {P0_11, LPC_TIM3_BASE, 3},
                                      {P1_22, LPC_TIM1_BASE, 3},
                                      {P1_25, LPC_TIM1_BASE, 3},
                                      {P1_28, LPC_TIM0_BASE, 3},
                                      {P1_29, LPC_TIM0_BASE, 3},
                                      {P3_25, LPC_TIM0_BASE, 2},
                                      {P3_26, LPC_TIM0_BASE, 2},
                                      {P4_28, LPC_TIM2_BASE, 3},
                                      {P4_29, LPC_TIM2_BASE, 3},
                                      {NC, NC, 0}};

/**
 * @brief   Determine if a match interrupt is pending
 * @param   pTimer      : Pointer to timer IP register address
 * @param   matchnum    : Match interrupt number to check
 * @return  false if the interrupt is not pending, otherwise true
 * @note    Determine if the match interrupt for the passed timer and match
 * counter is pending.
 */
static inline bool timer_MatchPending(LPC_TIM_TypeDef* pTimer,
                                      int8_t matchnum) {
    return (bool)((pTimer->IR & TIMER_MATCH_INT(matchnum)) != 0);
}

/**
 * @brief   Determine if a capture interrupt is pending
 * @param   pTimer  : Pointer to timer IP register address
 * @param   capnum  : Capture interrupt number to check
 * @return  false if the interrupt is not pending, otherwise true
 * @note    Determine if the capture interrupt for the passed capture pin is
 * pending.
 */
static inline bool timer_CapturePending(LPC_TIM_TypeDef* pTimer,
                                        int8_t capnum) {
    return (bool)((pTimer->IR & TIMER_CAP_INT(capnum)) != 0);
}

/**
 * @brief   Clears a (pending) match interrupt
 * @param   pTimer      : Pointer to timer IP register address
 * @param   matchnum    : Match interrupt number to clear
 * @return  Nothing
 * @note    Clears a pending timer match interrupt.
 */
static inline void timer_ClearMatch(LPC_TIM_TypeDef* pTimer, int8_t matchnum) {
    pTimer->IR = TIMER_IR_CLR(matchnum);
}

/**
 * @brief   Clears a (pending) capture interrupt
 * @param   pTimer  : Pointer to timer IP register address
 * @param   capnum  : Capture interrupt number to clear
 * @return  Nothing
 * @note    Clears a pending timer capture interrupt.
 */
static inline void timer_ClearCapture(LPC_TIM_TypeDef* pTimer, int8_t capnum) {
    pTimer->IR = (0x10 << capnum);
}

/**
 * @brief   Enables the timer (starts count)
 * @param   pTimer  : Pointer to timer IP register address
 * @return  Nothing
 * @note    Enables the timer to start counting.
 */
static inline void timer_Enable(LPC_TIM_TypeDef* pTimer) {
    pTimer->TCR |= TIMER_ENABLE;
}

/**
 * @brief   Disables the timer (stops count)
 * @param   pTimer  : Pointer to timer IP register address
 * @return  Nothing
 * @note    Disables the timer to stop counting.
 */
static inline void timer_Disable(LPC_TIM_TypeDef* pTimer) {
    pTimer->TCR &= ~TIMER_ENABLE;
}

/**
 * @brief   Returns the current timer count
 * @param   pTimer  : Pointer to timer IP register address
 * @return  Current timer terminal count value
 * @note    Returns the current timer terminal count.
 */
static inline uint32_t timer_ReadCount(LPC_TIM_TypeDef* pTimer) {
    return pTimer->TC;
}

/**
 * @brief  Returns the current prescale count
 * @param   pTimer  : Pointer to timer IP register address
 * @return Current timer prescale count value
 * @note    Returns the current prescale count.
 */
static inline uint32_t timer_ReadPrescale(LPC_TIM_TypeDef* pTimer) {
    return pTimer->PC;
}

/**
 * @brief   Sets the prescaler value
 * @param   pTimer      : Pointer to timer IP register address
 * @param   prescale    : Prescale value to set the prescale register to
 * @return  Nothing
 * @note    Sets the prescale count value.
 */
static inline void timer_PrescaleSet(LPC_TIM_TypeDef* pTimer,
                                     uint32_t prescale) {
    pTimer->PR = prescale;
}

/**
 * @brief   Sets a timer match value
 * @param   pTimer      : Pointer to timer IP register address
 * @param   matchnum    : Match timer to set match count for
 * @param   matchval    : Match value for the selected match count
 * @return  Nothing
 * @note    Sets ones of the timer match values.
 */
static inline void timer_SetMatch(LPC_TIM_TypeDef* pTimer, int8_t matchnum,
                                  uint32_t matchval) {
    *(&(pTimer->MR0) + (uint32_t)matchnum) = matchval;
}

/**
 * @brief   Reads a capture register
 * @param   pTimer  : Pointer to timer IP register address
 * @param   capnum  : Capture register to read
 * @return  The selected capture register value
 * @note    Returns the selected capture register value.
 */
static inline uint32_t timer_ReadCapture(LPC_TIM_TypeDef* pTimer,
                                         int8_t capnum) {
    // return pTimer->CR[capnum];
    return *(&(pTimer->CR0) + (uint32_t)capnum);
}

/**
 * @brief   Enables a match interrupt that fires when the terminal count
 *          matches the match counter value.
 * @param   pTimer      : Pointer to timer IP register address
 * @param   matchnum    : Match timer, 0 to 3
 * @return  Nothing
 */
static inline void timer_MatchEnableInt(LPC_TIM_TypeDef* pTimer,
                                        int8_t matchnum) {
    pTimer->MCR |= TIMER_INT_ON_MATCH(matchnum);
}

/**
 * @brief   Disables a match interrupt for a match counter.
 * @param   pTimer      : Pointer to timer IP register address
 * @param   matchnum    : Match timer, 0 to 3
 * @return  Nothing
 */
static inline void timer_MatchDisableInt(LPC_TIM_TypeDef* pTimer,
                                         int8_t matchnum) {
    pTimer->MCR &= ~TIMER_INT_ON_MATCH(matchnum);
}

/**
 * @brief   For the specific match counter, enables reset of the terminal count
 * register when a match occurs
 * @param   pTimer      : Pointer to timer IP register address
 * @param   matchnum    : Match timer, 0 to 3
 * @return  Nothing
 */
static inline void timer_ResetOnMatchEnable(LPC_TIM_TypeDef* pTimer,
                                            int8_t matchnum) {
    pTimer->MCR |= TIMER_RESET_ON_MATCH(matchnum);
}

/**
 * @brief   For the specific match counter, disables reset of the terminal count
 * register when a match occurs
 * @param   pTimer      : Pointer to timer IP register address
 * @param   matchnum    : Match timer, 0 to 3
 * @return  Nothing
 */
static inline void timer_ResetOnMatchDisable(LPC_TIM_TypeDef* pTimer,
                                             int8_t matchnum) {
    pTimer->MCR &= ~TIMER_RESET_ON_MATCH(matchnum);
}

/**
 * @brief   Enable a match timer to stop the terminal count when a
 *          match count equals the terminal count.
 * @param   pTimer      : Pointer to timer IP register address
 * @param   matchnum    : Match timer, 0 to 3
 * @return  Nothing
 */
static inline void timer_StopOnMatchEnable(LPC_TIM_TypeDef* pTimer,
                                           int8_t matchnum) {
    pTimer->MCR |= TIMER_STOP_ON_MATCH(matchnum);
}

/**
 * @brief   Disable stop on match for a match timer. Disables a match timer
 *          to stop the terminal count when a match count equals the terminal
 * count.
 * @param   pTimer      : Pointer to timer IP register address
 * @param   matchnum    : Match timer, 0 to 3
 * @return  Nothing
 */
static inline void timer_StopOnMatchDisable(LPC_TIM_TypeDef* pTimer,
                                            int8_t matchnum) {
    pTimer->MCR &= ~TIMER_STOP_ON_MATCH(matchnum);
}

/**
 * @brief   Enables capture on on rising edge of selected CAP signal for the
 *          selected capture register, enables the selected CAPn.capnum signal
 * to load
 *          the capture register with the terminal coount on a rising edge.
 * @param   pTimer  : Pointer to timer IP register address
 * @param   capnum  : Capture signal/register to use
 * @return  Nothing
 */
static inline void timer_CaptureRisingEdgeEnable(LPC_TIM_TypeDef* pTimer,
                                                 int8_t capnum) {
    pTimer->CCR |= TIMER_CAP_RISING(capnum);
}

/**
 * @brief   Disables capture on on rising edge of selected CAP signal. For the
 *          selected capture register, disables the selected CAPn.capnum signal
 * to load
 *          the capture register with the terminal coount on a rising edge.
 * @param   pTimer  : Pointer to timer IP register address
 * @param   capnum  : Capture signal/register to use
 * @return  Nothing
 */
static inline void timer_CaptureRisingEdgeDisable(LPC_TIM_TypeDef* pTimer,
                                                  int8_t capnum) {
    pTimer->CCR &= ~TIMER_CAP_RISING(capnum);
}

/**
 * @brief   Enables capture on on falling edge of selected CAP signal. For the
 *          selected capture register, enables the selected CAPn.capnum signal
 * to load
 *          the capture register with the terminal coount on a falling edge.
 * @param   pTimer  : Pointer to timer IP register address
 * @param   capnum  : Capture signal/register to use
 * @return  Nothing
 */
static inline void timer_CaptureFallingEdgeEnable(LPC_TIM_TypeDef* pTimer,
                                                  int8_t capnum) {
    pTimer->CCR |= TIMER_CAP_FALLING(capnum);
}

/**
 * @brief   Disables capture on on falling edge of selected CAP signal. For the
 *          selected capture register, disables the selected CAPn.capnum signal
 * to load
 *          the capture register with the terminal coount on a falling edge.
 * @param   pTimer  : Pointer to timer IP register address
 * @param   capnum  : Capture signal/register to use
 * @return  Nothing
 */
static inline void timer_CaptureFallingEdgeDisable(LPC_TIM_TypeDef* pTimer,
                                                   int8_t capnum) {
    pTimer->CCR &= ~TIMER_CAP_FALLING(capnum);
}

/**
 * @brief   Enables interrupt on capture of selected CAP signal. For the
 *          selected capture register, an interrupt will be generated when the
 * enabled
 *          rising or falling edge on CAPn.capnum is detected.
 * @param   pTimer  : Pointer to timer IP register address
 * @param   capnum  : Capture signal/register to use
 * @return  Nothing
 */
static inline void timer_CaptureEnableInt(LPC_TIM_TypeDef* pTimer,
                                          int8_t capnum) {
    pTimer->CCR |= TIMER_INT_ON_CAP(capnum);
}

/**
 * @brief   Disables interrupt on capture of selected CAP signal
 * @param   pTimer  : Pointer to timer IP register address
 * @param   capnum  : Capture signal/register to use
 * @return  Nothing
 */
static inline void timer_CaptureDisableInt(LPC_TIM_TypeDef* pTimer,
                                           int8_t capnum) {
    pTimer->CCR &= ~TIMER_INT_ON_CAP(capnum);
}

/**
 * @brief   Resets the timer terminal and prescale counts to 0
 * @param   pTimer  : Pointer to timer IP register address
 * @return  Nothing
 */
void timer_Reset(LPC_TIM_TypeDef* pTimer);

/**
 * @brief   Sets external match control (MATn.matchnum) pin control
 * @param   pTimer          : Pointer to timer IP register address
 * @param   initial_state   : Initial state of the pin, high(1) or low(0)
 * @param   matchState      : Selects the match state for the pin
 * @param   matchnum        : MATn.matchnum signal to use
 * @return  Nothing
 * @note    For the pin selected with matchnum, sets the function of the pin
 * that occurs on
 * a terminal count match for the match count.
 */
void timer_ExtMatchControlSet(LPC_TIM_TypeDef* pTimer, int8_t initial_state,
                              TIMER_PIN_MATCH_STATE matchState,
                              int8_t matchnum);

/**
 * @brief   Sets timer count source and edge with the selected passed from
 * CapSrc
 * @param   pTimer  : Pointer to timer IP register address
 * @param   capSrc  : timer clock source and edge
 * @param   capnum  : CAPn.capnum pin to use (if used)
 * @return  Nothing
 * @note    If CapSrc selected a CAPn pin, select the specific CAPn pin with the
 * capnum value.
 */
void timer_SetCountClockSrc(LPC_TIM_TypeDef* pTimer, TIMER_CAP_SRC_STATE capSrc,
                            int8_t capnum);
