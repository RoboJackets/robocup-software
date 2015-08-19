
include(ExternalProject)

ExternalProject_Add(googletest
    GIT_REPOSITORY ${PROJECT_SOURCE_DIR}/third_party/googletest
    # disable install step
    INSTALL_COMMAND ""
)
set_target_properties(googletest PROPERTIES EXCLUDE_FROM_ALL TRUE)

# specify include dir
ExternalProject_Get_Property(googletest source_dir)
include_directories(${source_dir}/include)

# specify link directory where the built gtest libs are placed
ExternalProject_Get_Property(googletest binary_dir)
link_directories(${binary_dir})

set(GTEST_LIBRARIES
    libgtest.a
    libgtest_main.a
    pthread
)
