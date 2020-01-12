# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if("C:/Users/thoma/OneDrive/RoboJackets/Repos/robocup-software/external/googletest/googletest" STREQUAL "")
  message(FATAL_ERROR "LOCAL can't be empty")
endif()

if(NOT EXISTS "C:/Users/thoma/OneDrive/RoboJackets/Repos/robocup-software/external/googletest/googletest")
  message(FATAL_ERROR "File not found: C:/Users/thoma/OneDrive/RoboJackets/Repos/robocup-software/external/googletest/googletest")
endif()

if("" STREQUAL "")
  message(WARNING "File will not be verified since no URL_HASH specified")
  return()
endif()

if("" STREQUAL "")
  message(FATAL_ERROR "EXPECT_VALUE can't be empty")
endif()

message(STATUS "verifying file...
     file='C:/Users/thoma/OneDrive/RoboJackets/Repos/robocup-software/external/googletest/googletest'")

file("" "C:/Users/thoma/OneDrive/RoboJackets/Repos/robocup-software/external/googletest/googletest" actual_value)

if(NOT "${actual_value}" STREQUAL "")
  message(FATAL_ERROR "error:  hash of
  C:/Users/thoma/OneDrive/RoboJackets/Repos/robocup-software/external/googletest/googletest
does not match expected value
  expected: ''
    actual: '${actual_value}'
")
endif()

message(STATUS "verifying file... done")
