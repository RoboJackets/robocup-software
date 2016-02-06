# mbed_util.cmake
# 
# Utility functions and constants for setting up the configuartion with the
# officially supported mbed SDK build system.
include(MbedTargets)


# Shows a listing of the major build configuration variables.
# Usage:
#   show_vars()
macro(SHOW_VARS)
    include(DumpVars)
endmacro()


# Assertion checks for build configuration debugging.
macro(ASSERT test comment)
    if(NOT ${test})
        message(FATAL_ERROR "Assertion failed: ${comment}")
    endif()
endmacro()


# Set the mbed platform we should compile for.
# Usage:
#   mbed_set_platform(<var-name-prefixt> TARGET <target-id>)
function(MBED_SET_PLATFORM target_var_base)
    set(__options "")
    set(__singleValueArgs TARGET)
    set(__multiValueArgs "")
    cmake_parse_arguments(MBED_SET "${__options}" "${__singleValueArgs}" "${__multiValueArgs}" ${ARGN})

    # make sure we are passed the vendor to use
    assert(MBED_SET_TARGET "TARGET is not set")

    # set the variable names for what target we are selecting
    set( MBED_TARGET_CORE_NAME           "MBED_TARGET_CORE_${MBED_SET_TARGET}"           )
    set( MBED_TARGET_ISA_NAME            "MBED_TARGET_ISA_${MBED_SET_TARGET}"            )
    set( MBED_TARGET_ARCH_NAME           "MBED_TARGET_ARCH_${MBED_SET_TARGET}"           )
    set( MBED_TARGET_VENDOR_NAME         "MBED_TARGET_VENDOR_${MBED_SET_TARGET}"         )
    set( MBED_TARGET_SERIES_NAME         "MBED_TARGET_SERIES_${MBED_SET_TARGET}"         )
    set( MBED_TARGET_LABELS_EXTRA_NAME   "MBED_TARGET_LABELS_EXTRA_${MBED_SET_TARGET}"   )
    set( MBED_TARGET_RTOS_ARCHS_NAME     "MBED_TARGET_RTOS_ARCHS_${MBED_SET_TARGET}"     )
    set( MBED_TARGET_TOOLCHAINS_NAME     "MBED_TARGET_TOOLCHAINS_${MBED_SET_TARGET}"     )
    set( MBED_TARGET_CODE_NAME           "MBED_TARGET_CODE_${MBED_SET_TARGET}"           )
    set( MBED_TARGET_PROGEN_NAME         "MBED_TARGET_PROGEN_${MBED_SET_TARGET}"         )
    set( MBED_TARGET_MACROS_NAME         "MBED_TARGET_MACROS_${MBED_SET_TARGET}"         )

    # set varibles for the target with a variable prefix name that was given to us
    set( ${target_var_base}_CORE          ${${MBED_TARGET_CORE_NAME}}          PARENT_SCOPE )
    set( ${target_var_base}_ISA           ${${MBED_TARGET_ISA_NAME}}           PARENT_SCOPE )
    set( ${target_var_base}_ARCH          ${${MBED_TARGET_ARCH_NAME}}          PARENT_SCOPE )
    set( ${target_var_base}_VENDOR        ${${MBED_TARGET_VENDOR_NAME}}        PARENT_SCOPE )
    set( ${target_var_base}_SERIES        ${${MBED_TARGET_SERIES_NAME}}        PARENT_SCOPE )
    set( ${target_var_base}_LABELS_EXTRA  ${${MBED_TARGET_LABELS_EXTRA_NAME}}  PARENT_SCOPE )
    set( ${target_var_base}_RTOS_ARCHS    ${${MBED_TARGET_RTOS_ARCHS_NAME}}    PARENT_SCOPE )
    set( ${target_var_base}_TOOLCHAINS    ${${MBED_TARGET_TOOLCHAINS_NAME}}    PARENT_SCOPE )
    set( ${target_var_base}_CODE          ${${MBED_TARGET_CODE_NAME}}          PARENT_SCOPE )
    set( ${target_var_base}_PROGEN        ${${MBED_TARGET_PROGEN_NAME}}        PARENT_SCOPE )
    set( ${target_var_base}_MACROS        ${${MBED_TARGET_MACROS_NAME}}        PARENT_SCOPE )

    # generate uppercase & lowercase versions of the target's core name
    # also replacing dashes in the uppercase name with underscores
    string(TOLOWER ${${MBED_TARGET_CORE_NAME}} lowerc)
    string(TOUPPER ${${MBED_TARGET_CORE_NAME}} upperc)
    string(REGEX REPLACE "-" "_" upperc     ${upperc})
    set( ${target_var_base}_CORE_LOWERC      ${lowerc}                       PARENT_SCOPE )
    set( ${target_var_base}_CORE_UPPERC      ${upperc}                       PARENT_SCOPE )

    # generate uppercase & lowercase versions of the target's core progen
    # also replacing dashes in the uppercase progen with underscores
    string(TOLOWER ${${MBED_TARGET_PROGEN_NAME}} lowerc)
    string(TOUPPER ${${MBED_TARGET_PROGEN_NAME}} upperc)
    string(REGEX REPLACE "-" "_" upperc         ${upperc})
    set( ${target_var_base}_PROGEN_LOWERC        ${lowerc}                   PARENT_SCOPE )
    set( ${target_var_base}_PROGEN_UPPERC        ${upperc}                   PARENT_SCOPE )
endfunction()


# Set the given variable to the paths to include for the ethernet library.
# Usage:
#   mbed_add_incs_eth(<directory-list> LIB_ROOT <mbed-build-tree> VENDOR <mbed-target-vendor>)
function(MBED_ADD_INCS_ETH dir_list)
    set(__options "")
    set(__singleValueArgs LIB_ROOT VENDOR)
    set(__multiValueArgs "")
    cmake_parse_arguments(MBED_ETH "${__options}" "${__singleValueArgs}" "${__multiValueArgs}" ${ARGN})

    # make sure we are passed the vendor to use
    assert(MBED_ETH_VENDOR "VENDOR is not set")

    set( ${dir_list}
        "${${dir_list}}"
        "${MBED_ETH_LIB_ROOT}/build/net/eth/EthernetInterface"
        "${MBED_ETH_LIB_ROOT}/build/net/eth/Socket"
        "${MBED_ETH_LIB_ROOT}/build/net/eth/lwip"
        "${MBED_ETH_LIB_ROOT}/build/net/eth/lwip/netif/ppp"
        "${MBED_ETH_LIB_ROOT}/build/net/eth/lwip/include/ipv4/lwip"
        "${MBED_ETH_LIB_ROOT}/build/net/eth/lwip/include/lwip"
        "${MBED_ETH_LIB_ROOT}/build/net/eth/lwip/include/netif"
        "${MBED_ETH_LIB_ROOT}/build/net/eth/lwip-sys/arch"
        "${MBED_ETH_LIB_ROOT}/build/net/eth/lwip-eth/arch/TARGET_${MBED_ETH_VENDOR}"
        PARENT_SCOPE
    )
endfunction()


# Set the given variable to the paths to include for the rtos library.
# Usage:
#   mbed_add_incs_rtos(<directory-list> LIB_ROOT <mbed-build-tree> ARCH <mbed-mcu-arch>)
function(MBED_ADD_INCS_RTOS dir_list)
    set(__options "")
    set(__singleValueArgs LIB_ROOT ARCH)
    set(__multiValueArgs "")
    cmake_parse_arguments(MBED_RTOS "${__options}" "${__singleValueArgs}" "${__multiValueArgs}" ${ARGN})

    # make sure we are passed the arch type to use
    assert(MBED_RTOS_ARCH "ARCH is not set")

    set( ${dir_list}
        "${${dir_list}}"
        "${MBED_RTOS_LIB_ROOT}/build/rtos"
        "${MBED_RTOS_LIB_ROOT}/build/rtos/TARGET_${MBED_RTOS_ARCH}"
        PARENT_SCOPE
    )
endfunction()


# Set the given variable to the paths to include for the usb libraries.
# Usage:
#   mbed_add_incs_usb(<directory-list> LIB_ROOT <mbed-build-tree>)
function(MBED_ADD_INCS_USB dir_list)
    set(__options)
    set(__singleValueArgs LIB_ROOT)
    set(__multiValueArgs "")
    cmake_parse_arguments(MBED_USB "${__options}" "${__singleValueArgs}" "${__multiValueArgs}" ${ARGN})

    set( ${dir_list}
        "${${dir_list}}"
        "${MBED_USB_LIB_ROOT}/build/usb/USBAudio"
        "${MBED_USB_LIB_ROOT}/build/usb/USBDevice"
        "${MBED_USB_LIB_ROOT}/build/usb/USBHID"
        "${MBED_USB_LIB_ROOT}/build/usb/USBMIDI"
        "${MBED_USB_LIB_ROOT}/build/usb/USBMSD"
        "${MBED_USB_LIB_ROOT}/build/usb/USBSerial"
        PARENT_SCOPE
    )
endfunction()


# Set the given variable to the paths to include for the usb libraries.
# Usage:
#   mbed_add_incs_usb_host(<directory-list> LIB_ROOT <mbed-build-tree>)
function(MBED_ADD_INCS_USB_HOST dir_list)
    set(__options)
    set(__singleValueArgs LIB_ROOT)
    set(__multiValueArgs "")
    cmake_parse_arguments(MBED_USB "${__options}" "${__singleValueArgs}" "${__multiValueArgs}" ${ARGN})

    set( ${dir_list}
        "${${dir_list}}"
        "${MBED_USB_LIB_ROOT}/build/usb_host/USBHost"
        "${MBED_USB_LIB_ROOT}/build/usb_host/USBHost3GModule"
        "${MBED_USB_LIB_ROOT}/build/usb_host/USBHostHID"
        "${MBED_USB_LIB_ROOT}/build/usb_host/USBHostHub"
        "${MBED_USB_LIB_ROOT}/build/usb_host/USBHostMIDI"
        "${MBED_USB_LIB_ROOT}/build/usb_host/USBHostMSD"
        "${MBED_USB_LIB_ROOT}/build/usb_host/USBHostSerial"
        PARENT_SCOPE
    )
endfunction()


# Set the given variable to the paths to include for the dsp library.
# Usage:
#   mbed_add_incs_dsp(<directory-list> LIB_ROOT <mbed-build-tree>)
function(MBED_ADD_INCS_DSP dir_list)
    set(__options "")
    set(__singleValueArgs LIB_ROOT)
    set(__multiValueArgs "")
    cmake_parse_arguments(MBED_DSP "${__options}" "${__singleValueArgs}" "${__multiValueArgs}" ${ARGN})

    set( ${dir_list}
        "${${dir_list}}"
        "${MBED_DSP_LIB_ROOT}/build/dsp"
        PARENT_SCOPE
    )
endfunction()


# Set the given variable to the paths to include for the rpc library.
# Usage:
#   mbed_add_incs_rpc(<directory-list> LIB_ROOT <mbed-build-tree>)
function(MBED_ADD_INCS_RPC dir_list)
    set(__options "")
    set(__singleValueArgs LIB_ROOT)
    set(__multiValueArgs "")
    cmake_parse_arguments(MBED_RPC "${__options}" "${__singleValueArgs}" "${__multiValueArgs}" ${ARGN})

    set( ${dir_list}
        "${${dir_list}}"
        "${MBED_RPC_LIB_ROOT}/build/rpc"
        PARENT_SCOPE
    )
endfunction()


# Set the given variable to the paths to include for ublox library.
# Usage:
#   mbed_add_incs_ublox(<directory-list> LIB_ROOT <mbed-build-tree>)
function(MBED_ADD_INCS_UBLOX dir_list)
    set(__options "")
    set(__singleValueArgs LIB_ROOT)
    set(__multiValueArgs "")
    cmake_parse_arguments(MBED_UBLOX "${__options}" "${__singleValueArgs}" "${__multiValueArgs}" ${ARGN})

    set( ${dir_list}
        "${${dir_list}}"
        "${MBED_UBLOX_LIB_ROOT}/build/ublox"
        PARENT_SCOPE
    )
endfunction()


# Set the given variable to the paths to include for the fat filesystem library.
# Usage:
#   mbed_add_incs_fatfs(<directory-list> LIB_ROOT <mbed-build-tree>)
function(MBED_ADD_INCS_FATFS dir_list)
    set(__options "")
    set(__singleValueArgs LIB_ROOT)
    set(__multiValueArgs "")
    cmake_parse_arguments(MBED_FATFS "${__options}" "${__oneValueArgs}" "${__multiValueArgs}" ${ARGN})

    set( ${dir_list}
        "${${dir_list}}"
        "${MBED_FATFS_LIB_ROOT}/build/fat"
        "${MBED_FATFS_LIB_ROOT}/build/fat/ChaN"
        PARENT_SCOPE
    )
endfunction()


# The following macro was taken from OpenCV's cmake utilies
# https://github.com/Itseez/opencv/blob/master/cmake/OpenCVUtils.cmake
# 
# Provides an option that the user can optionally select.
# Can accept condition to control when option is available for user.
# Usage:
#   rj_add_op(<option_variable> "help string describing the option" <initial value or boolean expression> [IF <condition>])
macro(RJ_ADD_OP variable description value)
    set(__value ${value})
    set(__condition "")
    set(__varname "__value")
    foreach(arg ${ARGN})
        if(arg STREQUAL "IF" OR arg STREQUAL "if")
            set(__varname "__condition")
        else()
            list(APPEND ${__varname} ${arg})
        endif()
    endforeach()
   unset(__varname)
    if(__condition STREQUAL "")
        set(__condition 2 GREATER 1)
    endif()

    if(${__condition})
        if(__value MATCHES ";")
            if(${__value})
                option(${variable} "${description}" ON)
            else()
                option(${variable} "${description}" OFF)
            endif()
        elseif(DEFINED ${__value})
            if(${__value})
                option(${variable} "${description}" ON)
            else()
                option(${variable} "${description}" OFF)
            endif()
        else()
            option(${variable} "${description}" ${__value})
        endif()
    else()
       unset(${variable} CACHE)
    endif()
   unset(__condition)
   unset(__value)
endmacro()
