/*
 *
 *
 */
#include "os.h"
#include "os_cfg_app.h"
#include "board.h"
#include "chip.h"

/** @defgroup uCos-III_43XX_M0_TICK LPC43xx M0 core tick function using RITimer
 * @ingroup RTOS_UCOSIII
 * This driver adds support for the RITimer for the uCos-III tick on the LPC43xx.
 * The M0 core in LPC43XX series of controllers does not have internal systick timer,
 * so the RITIMER is used to generate the RTOS ticks.
 * @{
 */

/**
 * @}
 */

#define CONFIG_CPU_CLOCK_HZ ( 204000000 )

#define RITENCLR               (1 << 1)
#define RITINT                 (1 << 0)

/* Timer reload value for next tick */
static CPU_INT32U reload_val;

/*$PAGE*/
/*
 *********************************************************************************************************
 *                                          SYS TICK HANDLER
 *
 * Description: Handle the system tick (SysTick) interrupt, which is used to generate the uC/OS-III tick
 *              interrupt.Since M0 is not implemented with systick RI timer is used.
 *
 * Arguments  : None.
 *
 *
 *********************************************************************************************************
 */

void  RIT_IRQHandler(void)
{

	CPU_SR_ALLOC();

	/* TODO: check if WWDT interrupt and redirect */
	Chip_RIT_ClearInt(LPC_RITIMER);
	Chip_RIT_SetCOMPVAL(LPC_RITIMER, Chip_RIT_GetCounter(LPC_RITIMER) + reload_val);/* Reload value */

	CPU_CRITICAL_ENTER();
	OSIntNestingCtr++;										/* Tell uC/OS-III that we are starting an ISR             */
	CPU_CRITICAL_EXIT();

	OSTimeTick();											/* Call uC/OS-III's OSTimeTick()                          */

	OSIntExit();											/* Tell uC/OS-III that we are leaving the ISR             */
}

void  OS_CSP_TickInit(void)
{
	/* Clear any pending interrupt */
	Chip_RIT_ClearInt(LPC_RITIMER);

	/* Calculate reload value */
	reload_val = (SystemCoreClock / OSCfg_TickRate_Hz);
	Chip_RIT_SetCOMPVAL(LPC_RITIMER, Chip_RIT_GetCounter(LPC_RITIMER) + reload_val);/* Start tick */

	NVIC_SetPriority((IRQn_Type) RITIMER_IRQn, ((0x01 << 3) | 0x01));
	NVIC_EnableIRQ((IRQn_Type) RITIMER_IRQn);
}

/*$PAGE*/
/*
 *********************************************************************************************************
 *                                          CPU_TS_TmrInit()
 *
 * Description : Initialize & start CPU timestamp timer.
 *
 * Argument(s) : none.
 *
 * Return(s)   : none.
 *
 * Caller(s)   : CPU_TS_Init().
 *
 *               This function is an INTERNAL CPU module function & MUST be implemented by application/
 *               BSP function(s) [see Note #1] but MUST NOT be called by application function(s).
 *
 * Note(s)     : (1) CPU_TS_TmrInit() is an application/BSP function that MUST be defined by the developer
 *                   if either of the following CPU features is enabled :
 *
 *                   (a) CPU timestamps
 *                   (b) CPU interrupts disabled time measurements
 *
 *                   See 'cpu_cfg.h  CPU TIMESTAMP CONFIGURATION  Note #1'
 *                     & 'cpu_cfg.h  CPU INTERRUPTS DISABLED TIME MEASUREMENT CONFIGURATION  Note #1a'.
 *
 *               (2) (a) Timer count values MUST be returned via word-size-configurable 'CPU_TS_TMR'
 *                       data type.
 *
 *                       (1) If timer has more bits, truncate timer values' higher-order bits greater
 *                           than the configured 'CPU_TS_TMR' timestamp timer data type word size.
 *
 *                       (2) Since the timer MUST NOT have less bits than the configured 'CPU_TS_TMR'
 *                           timestamp timer data type word size; 'CPU_CFG_TS_TMR_SIZE' MUST be
 *                           configured so that ALL bits in 'CPU_TS_TMR' data type are significant.
 *
 *                           In other words, if timer size is not a binary-multiple of 8-bit octets
 *                           (e.g. 20-bits or even 24-bits), then the next lower, binary-multiple
 *                           octet word size SHOULD be configured (e.g. to 16-bits).  However, the
 *                           minimum supported word size for CPU timestamp timers is 8-bits.
 *
 *                       See also 'cpu_cfg.h   CPU TIMESTAMP CONFIGURATION  Note #2'
 *                              & 'cpu_core.h  CPU TIMESTAMP DATA TYPES     Note #1'.
 *
 *                   (b) Timer SHOULD be an 'up'  counter whose values increase with each time count.
 *
 *                   (c) When applicable, timer period SHOULD be less than the typical measured time
 *                       but MUST be less than the maximum measured time; otherwise, timer resolution
 *                       inadequate to measure desired times.
 *
 *                   See also 'CPU_TS_TmrRd()  Note #2'.
 *********************************************************************************************************
 */

#if (CPU_CFG_TS_TMR_EN == DEF_ENABLED)
void  CPU_TS_TmrInit(void)
{
	CPU_INT32U fclk_freq;

	/* Use timer 1 */
	Chip_TIMER_Init(LPC_TIMER1);
	Chip_RGU_TriggerReset(RGU_TIMER1_RST);
	while (Chip_RGU_InReset(RGU_TIMER1_RST)) ;

	/* Get timer 1 peripheral clock rate */
	fclk_freq = (CPU_INT32U) Chip_Clock_GetRate(CLK_MX_TIMER1);

	/* Timer will recycle after 1s */
	Chip_TIMER_Reset(LPC_TIMER1);
	Chip_TIMER_SetMatch(LPC_TIMER1, 1, fclk_freq);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1, 1);
	Chip_TIMER_Enable(LPC_TIMER1);

	CPU_TS_TmrFreqSet((CPU_TS_TMR_FREQ) fclk_freq);
}

#endif

/*$PAGE*/
/*
 *********************************************************************************************************
 *                                           CPU_TS_TmrRd()
 *
 * Description : Get current CPU timestamp timer count value.
 *
 * Argument(s) : none.
 *
 * Return(s)   : Timestamp timer count (see Notes #2a & #2b).
 *
 * Caller(s)   : CPU_TS_Init(),
 *               CPU_TS_Get32(),
 *               CPU_TS_Get64(),
 *               CPU_IntDisMeasStart(),
 *               CPU_IntDisMeasStop().
 *
 *               This function is an INTERNAL CPU module function & MUST be implemented by application/
 *               BSP function(s) [see Note #1] but SHOULD NOT be called by application function(s).
 *
 * Note(s)     : (1) CPU_TS_TmrRd() is an application/BSP function that MUST be defined by the developer
 *                   if either of the following CPU features is enabled :
 *
 *                   (a) CPU timestamps
 *                   (b) CPU interrupts disabled time measurements
 *
 *                   See 'cpu_cfg.h  CPU TIMESTAMP CONFIGURATION  Note #1'
 *                     & 'cpu_cfg.h  CPU INTERRUPTS DISABLED TIME MEASUREMENT CONFIGURATION  Note #1a'.
 *
 *               (2) (a) Timer count values MUST be returned via word-size-configurable 'CPU_TS_TMR'
 *                       data type.
 *
 *                       (1) If timer has more bits, truncate timer values' higher-order bits greater
 *                           than the configured 'CPU_TS_TMR' timestamp timer data type word size.
 *
 *                       (2) Since the timer MUST NOT have less bits than the configured 'CPU_TS_TMR'
 *                           timestamp timer data type word size; 'CPU_CFG_TS_TMR_SIZE' MUST be
 *                           configured so that ALL bits in 'CPU_TS_TMR' data type are significant.
 *
 *                           In other words, if timer size is not a binary-multiple of 8-bit octets
 *                           (e.g. 20-bits or even 24-bits), then the next lower, binary-multiple
 *                           octet word size SHOULD be configured (e.g. to 16-bits).  However, the
 *                           minimum supported word size for CPU timestamp timers is 8-bits.
 *
 *                       See also 'cpu_cfg.h   CPU TIMESTAMP CONFIGURATION  Note #2'
 *                              & 'cpu_core.h  CPU TIMESTAMP DATA TYPES     Note #1'.
 *
 *                   (b) Timer SHOULD be an 'up'  counter whose values increase with each time count.
 *
 *                       (1) If timer is a 'down' counter whose values decrease with each time count,
 *                           then the returned timer value MUST be ones-complemented.
 *
 *                   (c) (1) When applicable, the amount of time measured by CPU timestamps is
 *                           calculated by either of the following equations :
 *
 *                           (A) Time measured  =  Number timer counts  *  Timer period
 *
 *                                   where
 *
 *                                       Number timer counts     Number of timer counts measured
 *                                       Timer period            Timer's period in some units of
 *                                                                   (fractional) seconds
 *                                       Time measured           Amount of time measured, in same
 *                                                                   units of (fractional) seconds
 *                                                                   as the Timer period
 *
 *                                                  Number timer counts
 *                           (B) Time measured  =  ---------------------
 *                                                    Timer frequency
 *
 *                                   where
 *
 *                                       Number timer counts     Number of timer counts measured
 *                                       Timer frequency         Timer's frequency in some units
 *                                                                   of counts per second
 *                                       Time measured           Amount of time measured, in seconds
 *
 *                       (2) Timer period SHOULD be less than the typical measured time but MUST be less
 *                           than the maximum measured time; otherwise, timer resolution inadequate to
 *                           measure desired times.
 *********************************************************************************************************
 */

#if (CPU_CFG_TS_TMR_EN == DEF_ENABLED)
CPU_TS_TMR  CPU_TS_TmrRd(void)
{
	CPU_TS_TMR  ts_tmr_cnts;

	ts_tmr_cnts  = (CPU_TS_TMR) Chip_TIMER_ReadCount(LPC_TIMER1);

	return ts_tmr_cnts;
}

#endif

#if 0
/*$PAGE*/
/*
 *********************************************************************************************************
 *                                         CPU_TSxx_to_uSec()
 *
 * Description : Convert a 32-/64-bit CPU timestamp from timer counts to microseconds.
 *
 * Argument(s) : ts_cnts   CPU timestamp (in timestamp timer counts [see Note #2aA]).
 *
 * Return(s)   : Converted CPU timestamp (in microseconds           [see Note #2aD]).
 *
 * Caller(s)   : Application.
 *
 *               This function is an (optional) CPU module application interface (API) function which
 *               MAY be implemented by application/BSP function(s) [see Note #1] & MAY be called by
 *               application function(s).
 *
 * Note(s)     : (1) CPU_TS32_to_uSec()/CPU_TS64_to_uSec() are application/BSP functions that MAY be
 *                   optionally defined by the developer when either of the following CPU features is
 *                   enabled :
 *
 *                   (a) CPU timestamps
 *                   (b) CPU interrupts disabled time measurements
 *
 *                   See 'cpu_cfg.h  CPU TIMESTAMP CONFIGURATION  Note #1'
 *                     & 'cpu_cfg.h  CPU INTERRUPTS DISABLED TIME MEASUREMENT CONFIGURATION  Note #1a'.
 *
 *               (2) (a) The amount of time measured by CPU timestamps is calculated by either of
 *                       the following equations :
 *
 *                                                                        10^6 microseconds
 *                       (1) Time measured  =   Number timer counts   *  -------------------  *  Timer period
 *                                                                            1 second
 *
 *                                              Number timer counts       10^6 microseconds
 *                       (2) Time measured  =  ---------------------  *  -------------------
 *                                                Timer frequency             1 second
 *
 *                               where
 *
 *                                   (A) Number timer counts     Number of timer counts measured
 *                                   (B) Timer frequency         Timer's frequency in some units
 *                                                                   of counts per second
 *                                   (C) Timer period            Timer's period in some units of
 *                                                                   (fractional)  seconds
 *                                   (D) Time measured           Amount of time measured,
 *                                                                   in microseconds
 *
 *                   (b) Timer period SHOULD be less than the typical measured time but MUST be less
 *                       than the maximum measured time; otherwise, timer resolution inadequate to
 *                       measure desired times.
 *
 *                   (c) Specific implementations may convert any number of CPU_TS32 or CPU_TS64 bits
 *                       -- up to 32 or 64, respectively -- into microseconds.
 *********************************************************************************************************
 */

#if (CPU_CFG_TS_32_EN == DEF_ENABLED)
CPU_INT64U  CPU_TS32_to_uSec(CPU_TS32  ts_cnts)
{

	return 0u;

}

#endif

#if (CPU_CFG_TS_64_EN == DEF_ENABLED)
CPU_INT64U  CPU_TS64_to_uSec(CPU_TS64  ts_cnts)
{

	return 0u;

}

#endif
#endif
