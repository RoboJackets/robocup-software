# ======================================================================
# Set Sources
# ======================================================================
set(RJ_PARAM_UTILS_SRCS
    param.cpp
    ros2_param_provider.cpp
    ros2_local_param_provider.cpp
    ros2_global_param_provider.cpp)

# ======================================================================
# Add sources
# ======================================================================
target_sources(rj_param_utils PRIVATE ${RJ_PARAM_UTILS_SRCS})
