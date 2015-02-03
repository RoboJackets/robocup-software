
set(CMAKE_SYSTEM_NAME "Atmel AVR ATtiny")

find_program(AVR_CC avr-gcc)
set(CMAKE_C_COMPILER ${AVR_CC})

# suppress "-rdynamic".  See http://www.cmake.org/pipermail/cmake/2007-October/016784.html
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
