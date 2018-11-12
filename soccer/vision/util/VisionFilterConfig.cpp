#include "VisionFilterConfig.hpp"

void VisionFilterConfig::createConfiguration(Configuration* cfg) {
    vision_loop_dt = new ConfigDouble("VisionFilter/loop_dt", 1.0 / 100.0);

    // The health of the kalman filter is a measure of how often it's updated
    // compared to the amount it's being predicted. Each new observation,
    // we increment the health, each prediction, we decrement it
    //
    // Min must be > 0
    filter_health_init = new ConfigInt("VisionFilter/Health/init", 2);
    filter_health_inc = new ConfigInt("VisionFilter/Health/increment", 2);
    filter_health_dec = new ConfigInt("VisionFilter/Health/decrement", 1);
    filter_health_max = new ConfigInt("VisionFilter/Health/max", 20);
    filter_health_min = new ConfigInt("VisionFilter/Health/min", 1);

    slow_kick_detector_history_length = new ConfigInt("VisionFilter/Kick/Detector/slow_hist_length", 5);
}