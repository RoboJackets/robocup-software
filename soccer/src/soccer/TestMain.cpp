#include "Configuration.hpp"
#include "gtest/gtest.h"

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);

    // setup config system because some tests rely on it
    std::shared_ptr<Configuration> config =
        Configuration::FromRegisteredConfigurables();

    return RUN_ALL_TESTS();
}
