# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_soccer_gui LANGUAGES CXX)

# ======================================================================
# Packaging
# ======================================================================
include(run_setup_py)
run_setup_py(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_INSTALL_PREFIX})

