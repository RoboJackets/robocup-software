# we'll go ahead and set the AVR toolchain cmake file too for consisteny in defining all toolchain setup files
set(ATTINY_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/attiny_toolchain.cmake)
set(ATTINY_TOOLCHAIN_FILE ${ATTINY_TOOLCHAIN_FILE} PARENT_SCOPE)

add_subdirectory(src-ctrl)
add_subdirectory(src-kckr)
add_subdirectory(src-fpga)
add_subdirectory(hw-test)
