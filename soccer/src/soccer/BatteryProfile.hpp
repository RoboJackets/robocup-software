#pragma once

#include <vector>
#include <stdarg.h>

/**
 * Calculates a battery's charge level given its voltage using a model
 * of its discharge curve.  Note that this is not an optimal way of measuring
 * remaining battery capacity.  It would be better to base it off of how much
 * current
 * has been drained from the battery so far, but we don't currently measure
 * that, so
 * we approximate charge level from voltage.
 */
class BatteryProfile {
public:
    /// Each Entry in the profile is a <voltage, charge level> pair.
    using Entry = std::pair<double, double>;

    /**
     * @brief Construct a battery profile with the given <voltage, charge level>
     * pairs.
     * These MUST be sorted in order of increasing voltage.
     */
    BatteryProfile(std::vector<BatteryProfile::Entry> dataPoints);

    /**
     * @brief Get the charge level given the voltage
     * @details The voltage curve is represented by a piecewise
     * linear graph based on the data points given in the constructor.
     *
     * @param voltage The voltage of the battery
     * @return A value between 0 and 1 indicating charge level
     */
    double getChargeLevel(double voltage) const;

private:
    std::vector<BatteryProfile::Entry> _dataPoints;
};

//  profiles for our batteries.  See BatteryProfile.cpp for more info
const extern BatteryProfile RJ2008BatteryProfile;
const extern BatteryProfile RJ2015BatteryProfile;
