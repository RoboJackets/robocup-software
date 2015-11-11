# MODDMA GPDMA Controller New features: transfer pins to memory buffer
# under periodic timer control and send double buffers to DAC

message(STATUS "downloading source files for the MODDMA library")

ExternalProject_Add(moddma_library
    HG_REPOSITORY       https://developer.mbed.org/users/AjK/code/MODDMA
    HG_TAG              17:97a16bf2ff43
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
    UPDATE_COMMAND      ""
)
set_target_properties(moddma_library PROPERTIES EXCLUDE_FROM_ALL TRUE)

# the directory to include for linking in with the common2015 library
ExternalProject_Get_Property(moddma_library SOURCE_DIR)

# the source files that will be added to common2015
set(MODDMA_SRC ${SOURCE_DIR}/MODDMA.cpp)
list(APPEND MODDMA_SRC ${SOURCE_DIR}/INIT.cpp)
list(APPEND MODDMA_SRC ${SOURCE_DIR}/SETUP.cpp)
list(APPEND MODDMA_SRC ${SOURCE_DIR}/DATALUTS.cpp)

# add the external project's path/src info into the accessory library lists
set(MBED_ASSEC_LIBS_INCLUDES    ${MBED_ASSEC_LIBS_INCLUDES} ${SOURCE_DIR}     )
set(MBED_ASSEC_LIBS_SRCS        ${MBED_ASSEC_LIBS_SRCS}     ${MODDMA_SRC}     )
set(MBED_ASSEC_LIBS_DEPENDS     ${MBED_ASSEC_LIBS_DEPENDS}  moddma_library    )
