#include "BatteryProfile.hpp"
#include <algorithm>

using namespace std;


//  2008,2011 battery: http://www.thunderpowerrc.com/Products/1350ProlitePlusPower/TP1350-4SP25_2
//  2015 battery: http://www.hobbyking.com/%E2%80%A6/__9942__ZIPPY_Flightmax_2200mAh_3S1P_40C.html
//
//  based on the li-po discharge curve from SparkFun
//  https://learn.sparkfun.com/tutorials/battery-technologies/lithium-polymer
const BatteryProfile RJ2008BatteryProfile(
    3,
    14.20, 0.20,
    15.10, 0.50,
    16.00, 1.00
);

#warning Battery profile for 2015 robot isnt calibrated yet - it always reports 0% charged
const BatteryProfile RJ2015BatteryProfile(
    2,
    0, 0.00,
    100, 0.00
);



BatteryProfile::BatteryProfile(int count, double v1, double l1, ...) {
    //  first data point
    _voltages.push_back(v1);
    _chargeLevels.push_back(l1);

    //  get the rest of the data points from the variadic list
    va_list vals;
    va_start(vals, l1);
    for (int i = 0; i < 2*(count - 1); i++) {
        _voltages.push_back(va_arg(vals, double));
        _chargeLevels.push_back(va_arg(vals, double));
    }
    va_end(vals);
}

BatteryProfile::BatteryProfile(const std::vector<double> &voltages, const std::vector<double> &chargeLevels) : _voltages(voltages), _chargeLevels(chargeLevels) {}

double BatteryProfile::getChargeLevel(double voltage) const {
    //  lower_bound does a binary search and returns an iterator pointing to the first element
    //  that is not less than @voltage, or end() if no element exists
    auto nextBiggest = lower_bound(_voltages.begin(), _voltages.end(), voltage);

    if (nextBiggest == _voltages.end()) {
        return 1;   //  this voltage is off the charts!
    } else if (nextBiggest == _voltages.begin()) {
        return 0;   //  this voltage is super low
    } else {
        int i = nextBiggest - _voltages.begin();
        double after = *nextBiggest;
        double before = *(nextBiggest-1);

        //  slope of this line segment
        double m = (_chargeLevels[i] - _chargeLevels[i-1]) / (after - before);

        //  y1-y2 = m(x1-x2)
        //  m(x1-x2) + y2 = y1
        return m*(voltage - before) + _chargeLevels[i-1];
    }
}
