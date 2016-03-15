# Control an array or chain of NeoPixel-compatible RGB LEDs. "NeoPixel" is
# Adafruit's name for WS2812- and WS2811-based addressable RGB LEDs. This
# library should work with any WS2811- or WS2812-based devices. Both the
# 400kHz and 800kHz protocols are supported. Most example code uses bit-banging
# to generate the timed signal precisely. This library uses an SPI peripheral
# instead. The main advantage of this is that the chip can service interrupts
# and the like without disrupting the signal (as long as the interrupts don't
# take _too_ long). The main disadvantage is that it requires the use of
# an SPI peripheral.

ExternalProject_Add(pixelarray_library
    HG_REPOSITORY       https://developer.mbed.org/users/JacobBramley/code/PixelArray
    HG_TAG              5:47802e75974e
    CONFIGURE_COMMAND   ""
    BUILD_COMMAND       ""
    INSTALL_COMMAND     ""
    UPDATE_COMMAND      ""
)
set_target_properties(pixelarray_library PROPERTIES EXCLUDE_FROM_ALL TRUE)

# the directory to include for linking in with the common2015 library
ExternalProject_Get_Property(pixelarray_library SOURCE_DIR)

# the source files that will be added to common2015
set(PIXARRY_ARC ${SOURCE_DIR}/neopixel.cpp)

# add the external project's path/src info into the accessory library lists
set(MBED_ASSEC_LIBS_INCLUDES    ${MBED_ASSEC_LIBS_INCLUDES} ${SOURCE_DIR}       )
set(MBED_ASSEC_LIBS_SRCS        ${MBED_ASSEC_LIBS_SRCS}     ${PIXARRY_ARC}      )
