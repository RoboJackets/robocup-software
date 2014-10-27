# This file instructs CMake to download the components of our ARM cross-compile toolchain and build them
# We use gcc, binutils, and newlib (a C std library)
#
# Note that EXCLUDE_FROM_ALL is set to TRUE for each of these - this ensures that we 
# don't build the toolchain unless explicitly told to or if something that depends on
# the toolchain is required
#
# I think gcc dropped support for the microprocessor we're using in our 2008 and 2011
# model robots, which is why we're using an old gcc version


include(ExternalProject)


#  package versions
set(BINUTILS_PACKAGE binutils-2.21.1)
set(GCC_VERSION 4.5.2)
set(NEWLIB_PACKAGE newlib-1.19.0)


# this forces CMake to add custom targets for the specified individual steps for each external project
# this lets us download and extract newlib, build gcc, then finish building newlib
set_property(DIRECTORY PROPERTY EP_STEP_TARGETS download configure build)


set(TOOLCHAIN_TARGET arm-elf)
set(TOOLCHAIN_PREFIX ${CMAKE_BINARY_DIR}/firmware/install)

# We want to add the toolchain install dir to the path so when building newlib,
# we use the newly-created crosscompile tools.  Unfortunately, ExternalProject doesn't 
# respect environment variables set in the calling CMakeLists file, so this doesn't work.
# Instead, we set PATH in each command that we need to use it.
# set(ENV{PATH} ${TOOLCHAIN_PREFIX}/bin:$ENV{PATH})


# note: we explicitly set the compiler in BUILD_COMMAND because by default it looks for 'arm-elf-cc', which doesn't exist
ExternalProject_Add(newlib
    URL ftp://sources.redhat.com/pub/newlib/${NEWLIB_PACKAGE}.tar.gz
    CONFIGURE_COMMAND PATH=${TOOLCHAIN_PREFIX}/bin:$ENV{PATH} CC=arm-elf-gcc
        ${CMAKE_CURRENT_BINARY_DIR}/newlib-prefix/src/newlib/configure
            --prefix=${TOOLCHAIN_PREFIX}
            --target=${TOOLCHAIN_TARGET}
            --enable-interwork
            --enable-multilib
            --disable-werror
    BUILD_COMMAND PATH=${TOOLCHAIN_PREFIX}/bin:$ENV{PATH} CC=arm-elf-gcc make
    INSTALL_COMMAND PATH=${TOOLCHAIN_PREFIX}/bin:$ENV{PATH} CC=arm-elf-gcc make install
)
set_target_properties(newlib PROPERTIES EXCLUDE_FROM_ALL TRUE)

ExternalProject_Add(gcc
    URL http://www.gtlib.gatech.edu/pub/gnu/gnu/gcc/gcc-${GCC_VERSION}/gcc-core-${GCC_VERSION}.tar.bz2
    URL_HASH MD5=AA9E36BEC080452372BFBA793428EE82
    CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/gcc-prefix/src/gcc/configure
        --prefix=${TOOLCHAIN_PREFIX}
        --target=${TOOLCHAIN_TARGET}
        --with-newlib
        --enable-languages=c
        --disable-threads
        --disable-shared
        --with-headers=${CMAKE_CURRENT_BINARY_DIR}/newlib-prefix/src/newlib/newlib/libc/include
        --enable-interwork
        --enable-multilib
        --with-system-zlib
        --with-mode=thumb
        --with-arch=armv4t
    && echo "MAKEINFO = :" >> ${CMAKE_CURRENT_BINARY_DIR}/gcc-prefix/src/gcc-build/Makefile
    BUILD_COMMAND make all-gcc
    INSTALL_COMMAND make install-gcc
)
set_target_properties(gcc PROPERTIES EXCLUDE_FROM_ALL TRUE)


ExternalProject_Add(binutils
    URL http://www.gtlib.gatech.edu/pub/gnu/gnu/binutils/${BINUTILS_PACKAGE}.tar.bz2
    URL_HASH MD5=BDE820EAC53FA3A8D8696667418557AD
    CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/binutils-prefix/src/binutils/configure
        --prefix=${TOOLCHAIN_PREFIX}
        --target=${TOOLCHAIN_TARGET}
        --enable-interwork
        --enable-multilib
        --disable-werror
    BUILD_COMMAND make
    INSTALL_COMMAND make install
)
set_target_properties(binutils PROPERTIES EXCLUDE_FROM_ALL TRUE)

add_dependencies(gcc binutils)
add_dependencies(gcc newlib-download)    # we use newlib headers when building gcc, so get newlib first
add_dependencies(newlib-configure gcc-build)


add_custom_target(arm-toolchain)
set_target_properties(arm-toolchain PROPERTIES EXCLUDE_FROM_ALL FALSE)  # remove this
add_dependencies(arm-toolchain gcc)
add_dependencies(arm-toolchain newlib)
add_dependencies(arm-toolchain binutils)
