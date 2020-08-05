# ======================================================================
# Set Sources
# ======================================================================
set(TEST_RJ_COMMON_SRCS # cmake-format: sortable
                        rj_common_convert_test.cpp)

# ======================================================================
# Add sources
# ======================================================================
target_sources(test_rj_common # cmake-format: sortable
               PRIVATE ${TEST_RJ_COMMON_SRCS})
