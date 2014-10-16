
set(BINUTILS_PACKAGE binutils-2.21.1)
set(GCC_VERSION 4.5.2)
set(NEWLIB_PACKAGE newlib-1.19.0)
set(TOOLCHAIN_TARGET arm-elf)


include(ExternalProject)


ExternalProject_Add(newlib
    URL ftp://sources.redhat.com/pub/newlib/${NEWLIB_PACKAGE}.tar.gz
    CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/newlib-prefix/src/newlib/configure
        --target=${TOOLCHAIN_TARGET}
        --enable-interwork
        --enable-multilib
        --disable-werror
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)
set_target_properties(newlib PROPERTIES EXCLUDE_FROM_ALL TRUE)

ExternalProject_Add(gcc
    URL http://www.gtlib.gatech.edu/pub/gnu/gnu/gcc/gcc-${GCC_VERSION}/gcc-core-${GCC_VERSION}.tar.bz2
    URL_HASH MD5=AA9E36BEC080452372BFBA793428EE82
    BUILD_COMMAND make
    CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/gcc-prefix/src/gcc/configure
        --prefix=${CMAKE_BINARY_DIR}
        --target=${TOOLCHAIN_TARGET}
        --with-newlib
        --enable-languages=c
        --disable-threads
        --disable-shared
        --with-headers=${CMAKE_CURRENT_BINARY_DIR}/newlib-prefix/src/${NEWLIB_PACKAGE}/newlib/libc/include
        --enable-interwork
        --enable-multilib
        --with-system-zlib
        --disable-doc
        --with-mode=thumb
        --with-arch=armv4t
    && echo "MAKEINFO = :" >> ${CMAKE_CURRENT_BINARY_DIR}/gcc-prefix/src/gcc-build/Makefile
)
set_target_properties(gcc PROPERTIES EXCLUDE_FROM_ALL TRUE)
add_dependencies(gcc newlib)    # we use newlib headers when building gcc, so get newlib first
add_dependencies(gcc binutils)


ExternalProject_Add(binutils
    URL http://www.gtlib.gatech.edu/pub/gnu/gnu/binutils/${BINUTILS_PACKAGE}.tar.bz2
    URL_HASH MD5=BDE820EAC53FA3A8D8696667418557AD
    CONFIGURE_COMMAND ${CMAKE_CURRENT_BINARY_DIR}/binutils-prefix/src/binutils/configure
        --target=${TOOLCHAIN_TARGET}
        --enable-interwork
        --enable-multilib
        --disable-werror
    BUILD_COMMAND make
    INSTALL_COMMAND ""
)
set_target_properties(binutils PROPERTIES EXCLUDE_FROM_ALL TRUE)



add_custom_target(arm-toolchain)
set_target_properties(arm-toolchain PROPERTIES EXCLUDE_FROM_ALL FALSE)  # remove this
add_dependencies(arm-toolchain gcc)
add_dependencies(arm-toolchain newlib)
add_dependencies(arm-toolchain binutils)
