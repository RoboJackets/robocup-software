
#test CMakeLists.txt for CV toys

project (CVTEST)
cmake_minimum_required(VERSION 2.6)

#add_executable (cvtest cvtest.cpp MotionFilter.cpp)
#add_executable (cvtest cvtest.cpp)
add_executable (cvtest cvtest.cpp KalmanFilter.cpp)
#add_executable (cvballdemo cvballdemo.cpp)
target_link_libraries(cvtest cv)