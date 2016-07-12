#include "power.h"
#include "timer.h"
#include "adc.h"
#include "status.h"
#include "control.h"
#include "sound.h"

int supply_raw, supply_min, supply_max;
int power_good_time;
int power_music_disable;

void power_init() {
    power_good_time = current_time;
    power_update();
}

// Called every update cycle to check for power failures
void power_update() {
    // Power failures are latched until reset.
    // Undervoltage could disappear briefly when load is removed, but the
    // battery is still weak.
    // Overvoltage indicates a hardware failure  (caps, TVS, battery resistance,
    // etc.) or a design flaw that
    // seriously needs to be fixed.

    supply_raw = adc[5];

    if (supply_raw < FUSE_BLOWN_RAW) {
        // If the battery voltage is very low, the motor fuse is probably blown
        // and we can't measure
        // the real battery voltage.
        //
        // We expect that it will not take long for the voltage to drop from
        // MIN_SUPPLY_RAW to FUSE_BLOWN_RAW,
        // or else we will get a spurious undervoltage indication.
        if ((current_time - power_good_time) >= 1000) {
            failures |= Fail_Fuse;
        }
    } else if (supply_raw < MIN_SUPPLY_RAW) {
        // If the battery voltage is too low for a long time, indicate an
        // undervoltage failure.
        // The time limit prevents beeping when the power switch is turned off
        // or when
        // accelerating with a weak but safe battery.
        if ((current_time - power_good_time) >= 1000) {
            failures |= Fail_Undervoltage;
        }
    } else {
        if (supply_raw > LOW_SUPPLY_RAW) {
            power_good_time = current_time;
        }

        // If the supply voltage ever gets too high, even briefly, there is a
        // potential for the
        // motor drivers to fail short.
        if (supply_raw > MAX_SUPPLY_RAW) {
            failures |= Fail_Overvoltage;
        }
    }

    // Keep track of minimum and maximum voltages since reset.
    if (supply_max == 0) {
        // First supply measurement
        supply_min = supply_raw;
        supply_max = supply_raw;
    } else {
        if (supply_raw < supply_min) {
            supply_min = supply_raw;
        }
        if (supply_raw > supply_max) {
            supply_max = supply_raw;
        }
    }

    // Stop driving if there is a power supply problem
    if (failures & Fail_Power) {
        controller = 0;
    }
}

// Called every main loop iteration to restart power-failure music
void power_fail_music() {
    if (!music_playing && !power_music_disable) {
        // The order that these are checked indicates the priority of the
        // failures:
        //   Overvoltage can damage the control electronics.
        //   A blown fuse indicates damage may have already occurred, and
        //   undervoltage can't be detected.
        //   Undervoltage indicates impending damage to the battery.
        if (failures & Fail_Overvoltage) {
            music_start(song_overvoltage);
        } else if (failures & Fail_Fuse) {
            music_start(song_fuse_blown);
        } else if ((failures & Fail_Undervoltage) ||
                   (supply_raw <= LOW_SUPPLY_RAW &&
                    (current_time - power_good_time) >= 1000)) {
            music_start(song_undervoltage);
        }
    }
}
