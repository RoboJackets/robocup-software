#include "gtest/gtest.h"

#include "Configuration.hpp"

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);

    // setup config system because some tests rely on it
    Configuration config;
    for (Configurable* obj : Configurable::configurables()) {
        obj->createConfiguration(&config);
    }

    return RUN_ALL_TESTS();
}
