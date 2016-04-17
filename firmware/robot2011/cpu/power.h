#pragma once

#define VBATT_RH 68000
#define VBATT_RL 10000

// Numerator and denominator for Vbatt raw-to-millivolts conversion ratio
#define VBATT_NUM (33 * (VBATT_RL + VBATT_RH))
#define VBATT_DIV (10230 * VBATT_RL / 1000)

#if VBATT_NUM >= (1 << 32) || VBATT_DIV >= (1 << 32)
#error VBATT_NUM out of range
#endif

#define MV_TO_RAW(v) (v * VBATT_DIV / VBATT_NUM)

// Supply voltage below which we believe that the motor fuse is blown.
#define FUSE_BLOWN_RAW MV_TO_RAW(2500)

// Minimum supply voltage, converted from millivolts.
// This is determined by the minimum safe discharge voltage of the LiPo pack.
#define MIN_SUPPLY_RAW MV_TO_RAW(13500)

// Voltage at which the low-battery music starts playing
#define LOW_SUPPLY_RAW MV_TO_RAW(14300)

// Maximum supply voltage, converted from millivolts.
// This is determined by the maximum voltage that the TC4428's can tolerate.
// This has a large safety margin because we may not detect spikes from
// regenerative
// braking quickly enough to protect the circuitry.
#define MAX_SUPPLY_RAW MV_TO_RAW(18000)

// Raw power supply voltage measurements:
// Last, minimum since reset, maximum since reset
extern int supply_raw, supply_min, supply_max;

// Last time the supply voltage was above MIN_SUPPLY_RAW
extern int power_good_time;

// If nonzero, power failure music will not continue playing
extern int power_music_disable;

void power_init(void);
void power_update(void);
void power_fail_music(void);
