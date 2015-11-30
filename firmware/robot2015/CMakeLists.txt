# Find the assembly source files and make sure they're compiled using the C compiler
enable_language(C ASM)
file(GLOB_RECURSE asm_SRC "*.S")
set_property(SOURCE ${asm_SRC} PROPERTY LANGUAGE C)
set_property(SOURCE ${asm_SRC} PROPERTY COMPILE_FLAGS "-x assembler-with-cpp")
set(CMAKE_ASM_FLAGS         "-x assembler-with-cpp")

# include all sub directories
add_subdirectory(src-ctrl)
add_subdirectory(src-kckr)
add_subdirectory(src-fpga)
add_subdirectory(hw-test)
