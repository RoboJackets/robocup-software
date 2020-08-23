# ======================================================================
# Preamble
# ======================================================================
cmake_minimum_required(VERSION 3.16)
project(rj_robocup_docs)

# ======================================================================
# Find package
# ======================================================================
find_package(Doxygen)
find_package(Sphinx)

if(NOT (${Doxygen_FOUND} AND ${Sphinx_FOUND}))
  message(STATUS "Doxygen and Sphinx not found. Not creating documentation targets.")
  return()
endif()

if (NOT BUILD_DOCUMENTATION)
    message(STATUS "BUILD_DOCUMENTATION option is not enabled. Not creating documentation targets.")
    return()
endif()

message(STATUS "Found Doxygen and Sphinx. Creating documentation targets.")
# ======================================================================
# Get all files to document.
# ======================================================================
set(PACKAGES soccer)
set(RJ_ROBOCUP_SOURCES)
foreach(package ${PACKAGES})
  file(
    GLOB_RECURSE
    ${package}_SOURCES
    "${CMAKE_SOURCE_DIR}/${package}/*.cpp"
    "${CMAKE_SOURCE_DIR}/${package}/*.h"
    "${CMAKE_SOURCE_DIR}/${package}/*.hpp")
  list(APPEND RJ_ROBOCUP_SOURCES ${${package}_SOURCES})
endforeach()

# ======================================================================
# Generate docs.
# ======================================================================
set(DOXYGEN_INPUT_DIR ${PROJECT_SOURCE_DIR}/../soccer)
set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/doxygen)
set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/xml/index.xml)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

# Replace variables inside @@ with the current values.
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

# Create outupt directory because Doxygen doesn't do it automatically.
file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})

# Only regenerate Doxygen when the Doxyfile or public headers change.
add_custom_command(
  OUTPUT ${DOXYGEN_INDEX_FILE}
  DEPENDS ${RJ_ROBOCUP_SOURCES}
  COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
  MAIN_DEPENDENCY ${DOXYFILE_OUT}
  ${DOXYFILE_IN}
  COMMENT "Generating docs")

add_custom_target(doxygen DEPENDS ${DOXYGEN_INDEX_FILE})

# ======================================================================
# Run Sphinx.
# ======================================================================
set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(SPHINX_BUILD ${CMAKE_CURRENT_BINARY_DIR}/docs_build/sphinx_docs)
set(SPHINX_INDEX_FILE ${SPHINX_BUILD}/index.html)

file(GLOB_RECURSE SPHINX_SOURCE_FILES ${SPHINX_SOURCE}/*)
message(STATUS "SPHINX_SOURCE_FILES: ${SPHINX_SOURCE_FILES}")

# Only regenerate Sphinx when: - Doxygen has rerun - Our doc files have been updated - The Sphinx
# config has been updated
add_custom_command(
  OUTPUT ${SPHINX_INDEX_FILE}
  COMMAND
    ${SPHINX_EXECUTABLE} -b html -W # Turn warnings into errors.
    --keep-going # Keep processing all errors before exiting.
    -j auto
    # Tell Breathe where to find the Doxygen output.
    -Dbreathe_projects.rj_robocup=${DOXYGEN_OUTPUT_DIR}/xml ${SPHINX_SOURCE} ${SPHINX_BUILD}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS ${SPHINX_SOURCE_FILES} ${DOXYGEN_INDEX_FILE}
  MAIN_DEPENDENCY ${SPHINX_SOURCE}/conf.py
  COMMENT "Generating documentation with Sphinx")

add_custom_target(sphinx DEPENDS ${SPHINX_INDEX_FILE})
