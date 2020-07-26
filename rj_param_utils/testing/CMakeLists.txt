#======================================================================
# Set Testing Sources
#======================================================================
set(RJ_PARAM_UTILS_TESTING_SRCS
    src/declare_test.cpp
    src/param_test.cpp)

#======================================================================
# Add Sources
#======================================================================
target_sources(rj_param_utils_testing PRIVATE ${RJ_PARAM_UTILS_TESTING_SRCS})

#======================================================================
# Include directories
#======================================================================
target_include_directories(rj_param_utils_testing PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
