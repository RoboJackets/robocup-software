#include "VisionFilterConfig.hpp"

REGISTER_CONFIGURABLE(VisionFilterConfig)

ConfigDouble* VisionFilterConfig::vision_loop_dt;

ConfigInt* VisionFilterConfig::max_num_cameras;

ConfigInt* VisionFilterConfig::filter_health_init;
ConfigInt* VisionFilterConfig::filter_health_inc;
ConfigInt* VisionFilterConfig::filter_health_dec;
ConfigInt* VisionFilterConfig::filter_health_max;
ConfigInt* VisionFilterConfig::filter_health_min;

ConfigInt* VisionFilterConfig::slow_kick_detector_history_length;
ConfigInt* VisionFilterConfig::fast_kick_detector_history_length;

void VisionFilterConfig::createConfiguration(Configuration* cfg) {
    vision_loop_dt = new ConfigDouble(cfg, "VisionFilter/loop_dt", 1.0 / 100.0);

    max_num_cameras = new ConfigInt(cfg, "VisionFilter/max_num_cameras", 12);

    // The health of the kalman filter is a measure of how often it's updated
    // compared to the amount it's being predicted. Each new observation,
    // we increment the health, each prediction, we decrement it
    //
    // Min must be > 0
    filter_health_init = new ConfigInt(cfg, "VisionFilter/Health/init", 2);
    filter_health_inc = new ConfigInt(cfg, "VisionFilter/Health/increment", 2);
    filter_health_dec = new ConfigInt(cfg, "VisionFilter/Health/decrement", 1);
    filter_health_max = new ConfigInt(cfg, "VisionFilter/Health/max", 20);
    filter_health_min = new ConfigInt(cfg, "VisionFilter/Health/min", 1);

    slow_kick_detector_history_length = new ConfigInt(cfg, "VisionFilter/Kick/Detector/slow_hist_length", 5);
    fast_kick_detector_history_length = new ConfigInt(cfg, "VisionFilter/Kick/Detector/fast_hist_length", 3);
}