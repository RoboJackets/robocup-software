# ======================================================================
# Set Sources
# ======================================================================
set(RJ_COMMON_SRCS # cmake-format: sortable
                   Field_Dimensions.cpp multicast.cpp RefereeEnums.cpp qt_utils.cpp)

# ======================================================================
# Add sources
# ======================================================================
target_sources(rj_common PRIVATE ${RJ_COMMON_SRCS})
