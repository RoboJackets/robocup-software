#pragma once

#include <Configuration.hpp>

class VisionFilterConfig {
public:
    static void createConfiguration(Configuration* cfg);

    static ConfigDouble* vision_loop_dt;

    static ConfigInt* filter_health_init;
    static ConfigInt* filter_health_inc;
    static ConfigInt* filter_health_dec;
    static ConfigInt* filter_health_max;
    static ConfigInt* filter_health_min;

    static ConfigInt* slow_kick_detector_history_length;
};