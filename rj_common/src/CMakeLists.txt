# ======================================================================
# Set Sources
# ======================================================================
set(RJ_COMMON_SRCS
    field_dimensions.cpp
    multicast.cpp
    referee_enums.cpp
    transforms.cpp
    qt_utils.cpp)

# ======================================================================
# Add sources
# ======================================================================
target_sources(rj_common PRIVATE ${RJ_COMMON_SRCS})
