# =============================================================================
# Mbed setup for interfacing into the official mbed SDK's build system.
# =============================================================================
CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)
include(MbedUtil)
include(ShowVars)
# ------------------------------------------------------------------------------
# turn build options on/off with there cmake options
rj_add_op( MBED_WITH_RTOS      "Include mbed RTOS support"                 ON    )
rj_add_op( MBED_WITH_RPC       "Include mbed RPC support"                  ON    )
rj_add_op( MBED_WITH_USB       "Include mbed USB support"                  OFF   )
rj_add_op( MBED_WITH_USB_HOST  "Include mbed USB Host support"             OFF   )
rj_add_op( MBED_WITH_ETH       "Include mbed ethernet/networking support"  OFF   )
rj_add_op( MBED_WITH_DSP       "Include mbed DSP support"                  OFF   )
rj_add_op( MBED_WITH_FATFS     "Include mbed FAT filesystem support"       OFF   )
rj_add_op( MBED_WITH_UBLOX     "Include mbed UBLOX support"                OFF   )

# turn on building the rtos library if ethernet/networking is set to be built
if(${MBED_WITH_ETH})
    set(MBED_WITH_RTOS ON)
endif()

# ------------------------------------------------------------------------------
# set the toolchain that the mbed SDK build system will use for compiling the
# official mbed libraries
set(MBED_TOOLCHAIN "GCC_ARM")

# ------------------------------------------------------------------------------
# set the platform that we are compiling for
set(MBED_PLATFORM "LPC1768")

# ------------------------------------------------------------------------------
# The following function will set the following variables to the selected
# target's parameters
# <variable_name>_CORE        
# <variable_name>_ISA         
# <variable_name>_ARCH        
# <variable_name>_VENDOR      
# <variable_name>_SERIES      
# <variable_name>_LABELS_EXTRA
# <variable_name>_RTOS_ARCHS  
# <variable_name>_TOOLCHAINS  
# <variable_name>_CODE        
# <variable_name>_PROGEN      
# <variable_name>_MACROS      
mbed_set_platform(MBED_TARGET TARGET ${MBED_PLATFORM})

# ------------------------------------------------------------------------------
# check to make sure the toolchain is supported for the target
list (FIND MBED_TARGET_TOOLCHAINS ${MBED_TOOLCHAIN} _index)
if (NOT ${_index} GREATER -1)
    message(FATAL_ERROR "unsupported toolchain: ${MBED_TOOLCHAIN}")
endif()

# ------------------------------------------------------------------------------
# git checkout and build location of mbed libraries
set(MBED_REPO_DIR "${CMAKE_CURRENT_BINARY_DIR}/mbed_libraries-prefix/src/mbed_libraries")

# ------------------------------------------------------------------------------
# generate upppercase/lowercase versions of the platform name
string(TOUPPER ${MBED_PLATFORM} MBED_PLATFORM_UPPERC)
string(TOLOWER ${MBED_PLATFORM} MBED_PLATFORM_LOWERC)

# ------------------------------------------------------------------------------
# compiler settings
set(COMMON_FLAGS " -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fno-exceptions -fno-builtin -MMD -fno-delete-null-pointer-checks")
set(COMMON_FLAGS "${COMMON_FLAGS} -mcpu=${MBED_TARGET_CORE_LOWERC} -O2 -mthumb -fno-exceptions -msoft-float -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")

# pass these defines to the preprocessor
set(COMMON_DEFINES "-DTARGET_${MBED_PLATFORM_UPPERC} -DTARGET_${MBED_TARGET_ISA} -DTARGET_${MBED_TARGET_VENDOR} -DTOOLCHAIN_${MBED_TOOLCHAIN}")

# add definitions for each macro from the official mbed SDK parameters
foreach(def ${MBED_TARGET_MACROS})
    add_definitions(-D${def})
endforeach()

SET(MBED_CMAKE_C_FLAGS      "${COMMON_FLAGS} ${CMAKE_CXX_FLAGS} ${COMMON_DEFINES} -std=gnu99")
SET(MBED_CMAKE_CXX_FLAGS    "${COMMON_FLAGS} ${CMAKE_CXX_FLAGS} ${COMMON_DEFINES}")

# ------------------------------------------------------------------------------
# libraries for mbed
set(MBED_LINK_LIBRARY stdc++ supc++ m gcc g c nosys rdimon)

# ------------------------------------------------------------------------------
# linker settings
set(MBED_CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -Wl,--wrap,main --specs=nosys.specs  -u _printf_float -u _scanf_float")
set(MBED_CMAKE_EXE_LINKER_FLAGS "${MBED_CMAKE_EXE_LINKER_FLAGS} -T '${MBED_REPO_DIR}/build/mbed/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/${MBED_PLATFORM}.ld' -static")

# ------------------------------------------------------------------------------
# mbed include directories for all targets
set(MBED_INC_DIRS           "${MBED_REPO_DIR}/build/mbed")
list(APPEND MBED_INC_DIRS   "${MBED_REPO_DIR}/build/mbed/TARGET_${MBED_PLATFORM_UPPERC}")
list(APPEND MBED_INC_DIRS   "${MBED_REPO_DIR}/build/mbed/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}")
list(APPEND MBED_INC_DIRS   "${MBED_REPO_DIR}/build/mbed/TARGET_${MBED_PLATFORM_UPPERC}/TARGET_${MBED_TARGET_VENDOR}/TARGET_${MBED_TARGET_SERIES}")
list(APPEND MBED_INC_DIRS   "${MBED_REPO_DIR}/build/mbed/TARGET_${MBED_PLATFORM_UPPERC}/TARGET_${MBED_TARGET_VENDOR}/TARGET_${MBED_TARGET_SERIES}/TARGET_${MBED_TARGET_PROGEN_UPPERC}")

# ------------------------------------------------------------------------------
# compile with support for the networking library
if(${MBED_WITH_ETH})
    mbed_add_incs_eth(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR} VENDOR ${MBED_TARGET_VENDOR})
    list(APPEND MBED_OPT_LIBS       "--eth")
    list(APPEND MBED_TARGET_OBJS    "${MBED_REPO_DIR}/build/eth/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/libeth.a")
    list(APPEND MBED_CMAKE_CXX_FLAGS "-Wno-literal-suffix")
endif()

# ------------------------------------------------------------------------------
# compile with support for the RTOS library
if(${MBED_WITH_RTOS})
    mbed_add_incs_rtos(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR} TARGET ${MBED_PLATFORM} ARCH ${MBED_TARGET_ARCH} TOOLCHAIN ${MBED_TOOLCHAIN})
    list(APPEND MBED_OPT_LIBS       "--rtos")
    list(APPEND MBED_TARGET_OBJS    "${MBED_REPO_DIR}/build/rtos/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/librtos.a")
    list(APPEND MBED_TARGET_OBJS    "${MBED_REPO_DIR}/build/rtos/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/librtx.a")
endif()

# ------------------------------------------------------------------------------
# compile with support for the USB library
if(${MBED_WITH_USB})
    mbed_add_incs_usb(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR})
    list(APPEND MBED_OPT_LIBS       "--usb")
    list(APPEND MBED_TARGET_OBJS    "${MBED_REPO_DIR}/build/usb/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/libUSBDevice.a")
endif()

# ------------------------------------------------------------------------------
# compile with support for the USB Host library
if(${MBED_WITH_USB_HOST})
    mbed_add_incs_usb_host(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR})
    list(APPEND MBED_OPT_LIBS       "--usb_host")
    list(APPEND MBED_TARGET_OBJS    "${MBED_REPO_DIR}/build/usb_host/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/libUSBHost.a")
endif()

# ------------------------------------------------------------------------------
# compile with support for the DSP library
if(${MBED_WITH_DSP})
    mbed_add_incs_dsp(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR})
    list(APPEND MBED_OPT_LIBS         "--dsp")
    list(APPEND MBED_TARGET_OBJS      "${MBED_REPO_DIR}/build/dsp/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/libcmsis_dsp.a")
    list(APPEND MBED_TARGET_OBJS      "${MBED_REPO_DIR}/build/dsp/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/libdsp.a")
endif()

# ------------------------------------------------------------------------------
# compile with support for the RPC library
if(${MBED_WITH_RPC})
    mbed_add_incs_rpc(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR})
    list(APPEND MBED_OPT_LIBS       "--rpc")
    list(APPEND MBED_TARGET_OBJS    "${MBED_REPO_DIR}/build/rpc/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/librpc.a")
endif()

# ------------------------------------------------------------------------------
# compile with support for the FAT filesystem library
if(${MBED_WITH_FATFS})
    mbed_add_incs_fatfs(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR})
    list(APPEND MBED_OPT_LIBS       "--fat")
    list(APPEND MBED_TARGET_OBJS    "${MBED_REPO_DIR}/build/fat/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/libfat.a")
endif()

# ------------------------------------------------------------------------------
# compile with support for the UBLOX library
if(${MBED_WITH_UBLOX})
    mbed_add_incs_ublox(MBED_INC_DIRS LIB_ROOT ${MBED_REPO_DIR})
    list(APPEND MBED_OPT_LIBS       "--ublox")
endif()

# ------------------------------------------------------------------------------
# official MBED libraries
include(ExternalProject)
ExternalProject_Add(mbed_libraries
    URL                 ${PROJECT_SOURCE_DIR}/external/mbed_lib_build_tools
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ${MBED_REPO_DIR}/workspace_tools/build.py
                                --mcu=${MBED_PLATFORM_UPPERC}
                                --tool=${MBED_TOOLCHAIN}
                                ${MBED_OPT_LIBS}
    INSTALL_COMMAND     ""
    UPDATE_COMMAND      ""
)
set_target_properties(mbed_libraries PROPERTIES EXCLUDE_FROM_ALL TRUE)

# find the names of all system objects to include
set(MBED_TARGET_SYS_OBJS "")
file(GLOB MBED_TARGET_SYSTEM_OBJS  LIST_DIRECTORIES false "${PROJECT_SOURCE_DIR}/external/mbed_lib_build_tools/libraries/mbed/targets/cmsis/TARGET_${MBED_TARGET_VENDOR}/TARGET_${MBED_TARGET_SERIES}/*.c")
foreach(target_sys_obj ${MBED_TARGET_SYSTEM_OBJS})
    # extract just the filename - excluding the extension so we can set it as an object file to include for later
    get_filename_component(basename ${target_sys_obj} NAME_WE)
    # append it to the list of objects to track dependencies for
    list(APPEND MBED_TARGET_SYS_OBJS "${MBED_REPO_DIR}/build/mbed/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/${basename}.o")
endforeach()

# find the name of the startup object to include
file(GLOB MBED_TARGET_STARTUP_OBJS LIST_DIRECTORIES false "${PROJECT_SOURCE_DIR}/external/mbed_lib_build_tools/libraries/mbed/targets/cmsis/TARGET_${MBED_TARGET_VENDOR}/TARGET_${MBED_TARGET_SERIES}/TOOLCHAIN_${MBED_TOOLCHAIN}/*.S")
foreach(target_sys_obj ${MBED_TARGET_STARTUP_OBJS})
    # do the same thing that we did above, except this time with the target's main startup file
    get_filename_component(basename ${target_sys_obj} NAME_WE)
    # append it to the list of objects to track dependencies for
    list(APPEND MBED_TARGET_SYS_OBJS "${MBED_REPO_DIR}/build/mbed/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/${basename}.o")
endforeach()

# append them to the list along with the other common target objects
list(APPEND MBED_TARGET_SYS_OBJS "${MBED_REPO_DIR}/build/mbed/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/board.o")
list(APPEND MBED_TARGET_SYS_OBJS "${MBED_REPO_DIR}/build/mbed/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/retarget.o")
set_source_files_properties(
    ${MBED_TARGET_SYS_OBJS} PROPERTIES
    EXTERNAL_OBJECT TRUE # Identifies this as an object file
    GENERATED TRUE # Avoids need for file to exist at configure-time
)

list(APPEND MBED_TARGET_OBJS "${MBED_REPO_DIR}/build/mbed/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/libmbed.a")

# tell CMake that the obj files all come from the ExternalProject
# otherwise it'll complain that the files can't be found
# ------------------------------------------------------------------------------
foreach(obj ${MBED_TARGET_OBJS})
    set_source_files_properties(
        ${obj} PROPERTIES
        EXTERNAL_OBJECT TRUE # Identifies this as an object file
        GENERATED TRUE # Avoids need for file to exist at configure-time
        DEPENDS mbed_libraries
    )
endforeach()

set_source_files_properties(
    "${MBED_REPO_DIR}/build/mbed/TARGET_${MBED_PLATFORM_UPPERC}/TOOLCHAIN_${MBED_TOOLCHAIN}/${MBED_PLATFORM}.ld" PROPERTIES
    EXTERNAL_OBJECT TRUE # Identifies this as an object file
    GENERATED TRUE # Avoids need for file to exist at configure-time
)

# include the paths to the headers
include_directories(${MBED_INC_DIRS})

# ------------------------------------------------------------------------------
# set some standaradized variables for the file that included this one to use if needed
set( MBED_INCLUDE_DIR  ${MBED_INC_DIRS}      )
set( MBED_LIBRARY      ${MBED_TARGET_SYS_OBJS} ${MBED_TARGET_OBJS} )
