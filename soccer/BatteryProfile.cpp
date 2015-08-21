#include "BatteryProfile.hpp"
#include <algorithm>

using namespace std;


//  2008,2011 battery: http://www.thunderpowerrc.com/Products/1350ProlitePlusPower/TP1350-4SP25_2
//  2015 battery: http://www.hobbyking.com/%E2%80%A6/__9942__ZIPPY_Flightmax_2200mAh_3S1P_40C.html
//
//  based on the li-po discharge curve from SparkFun
//  https://learn.sparkfun.com/tutorials/battery-technologies/lithium-polymer
const BatteryProfile RJ2008BatteryProfile(
    {14.20, 0.20},
    {15.10, 0.50},
    {16.00, 1.00}
);

#warning Battery profile for 2015 robot isnt calibrated yet - it always reports 0% charged
const BatteryProfile RJ2015BatteryProfile({
    {0, 0.00},
    {100, 0.00}
});


BatteryProfile::BatteryProfile(std::vector<BatteryProfile::Entry> dataPoints) {
    _dataPoints = std::move(dataPoints);
}

double BatteryProfile::getChargeLevel(double voltage) const {
    //  lower_bound does a binary search and returns an iterator pointing to the first element
    //  that is not less than @voltage, or end() if no element exists
    auto nextBiggest = lower_bound(
        _dataPoints.begin(), _dataPoints.end(), voltage,
        [](const Entry &entry, double val) { return entry.first < val; });

    if (nextBiggest == _dataPoints.end()) {
        return 1;   //  this voltage is off the charts!
    } else if (nextBiggest == _dataPoints.begin()) {
        return 0;   //  this voltage is super low
    } else {
        int i = nextBiggest - _dataPoints.begin();
        double after = nextBiggest->first;
        double before = (nextBiggest-1)->first;

        //  slope of this line segment
        double m = (_dataPoints[i].second - _dataPoints[i-1].second) / (after - before);

        //  y1-y2 = m(x1-x2)
        //  m(x1-x2) + y2 = y1
        return m*(voltage - before) + _dataPoints[i-1].second;
    }
}
