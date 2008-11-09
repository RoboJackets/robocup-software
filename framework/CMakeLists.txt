#Build Framework Driver

project(frameworkDriver)
cmake_minimum_required(VERSION 2.6)

add_executable(framework main.cpp DataInput.cpp WorldModel.cpp Soccer.cpp Motion.cpp Radio.cpp)