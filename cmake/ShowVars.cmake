# Shows a listing of the major build configuration variables.
# Usage:
#   show_vars()
macro(SHOW_VARS)
    get_cmake_property(_variableNames VARIABLES)
    foreach (_variableName ${_variableNames})
        string(REGEX MATCH "MBED_TARGET_|MBED_AVAILABLE_TARGETS.*" IS_MBED_MATCH "${_variableName}")
        if("${IS_MBED_MATCH}" STREQUAL "")
            string(REGEX MATCH "_eigen3_" IS_DEFINE_MATCH "${_variableName}")
            if("${IS_DEFINE_MATCH}" STREQUAL "")
                message(STATUS "${_variableName}\t=>\t${${_variableName}}")
            endif()
        endif()
    endforeach()
endmacro()
  
