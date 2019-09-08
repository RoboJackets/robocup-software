#pragma once

#include <Configuration.hpp>

/**
 * Contains all the global vision config variables
 */
class VisionFilterConfig {
public:
    VisionFilterConfig() = delete;
    ~VisionFilterConfig() = delete;

    static void createConfiguration(Configuration* cfg);

    // 1/freq of the vision loop
    static ConfigDouble* vision_loop_dt;

    // Max number of cameras possible on the field
    static ConfigInt* max_num_cameras;

    // Initial health of the kalman filters, must be between min and max
    static ConfigInt* filter_health_init;
    // How much to increment every measurement
    static ConfigInt* filter_health_inc;
    // How much to decrement every predict without measurement
    static ConfigInt* filter_health_dec;
    // Max health of the filters
    static ConfigInt* filter_health_max;
    // Minimum health of the filters, must be greater than 0
    static ConfigInt* filter_health_min;

    // How many frames to store for the slow kick detector
    static ConfigInt* slow_kick_detector_history_length;
    // How many frames to store for the fast kick detector
    static ConfigInt* fast_kick_detector_history_length;
};