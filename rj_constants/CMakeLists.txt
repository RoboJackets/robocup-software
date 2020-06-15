#======================================================================
# Preamble
#======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_constants LANGUAGES CXX)

#======================================================================
# Define Targets
#======================================================================
add_library(rj_constants INTERFACE)
target_include_directories(rj_constants INTERFACE include)

#======================================================================
# Packaging
#======================================================================
install(DIRECTORY include/
    DESTINATION include/)

#======================================================================
# Ament Packaging
#======================================================================
ament_export_include_directories(include)
ament_package()
