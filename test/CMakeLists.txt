
include(ExternalProject)

ExternalProject_Add(googletest
    GIT_REPOSITORY ${PROJECT_SOURCE_DIR}/third_party/googletest
    CMAKE_ARGS -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG:PATH=DebugLibs
               -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE:PATH=ReleaseLibs
    PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
    # disable install step
    INSTALL_COMMAND ""
)
set_target_properties(googletest PROPERTIES EXCLUDE_FROM_ALL TRUE)

# specify include dir
ExternalProject_Get_Property(googletest source_dir)
set(GTEST_INCLUDE_DIRS ${source_dir}/include PARENT_SCOPE)

# specify link libraries
ExternalProject_Get_Property(googletest binary_dir)
set(GTEST_LIBS_DIR ${binary_dir} PARENT_SCOPE)
