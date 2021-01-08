# ======================================================================
# Set Sources
# ======================================================================
set(RJ_COMMON_SRCS
    field_dimensions.cpp
    multicast.cpp
    referee_enums.cpp
    qt_utils.cpp
    team_color.cpp)

# ======================================================================
# Add sources
# ======================================================================
target_sources(rj_common PRIVATE ${RJ_COMMON_SRCS})
