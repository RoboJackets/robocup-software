ament_add_gtest(rj_param_utils_test 
                    src/declare_test.cpp 
                    src/global_param_provider_test.cpp 
                    src/param_test.cpp)

target_link_libraries(rj_param_utils_test ${PROJECT_NAME})
target_include_directories(rj_param_utils_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
