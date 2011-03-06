/**
 * Main test runner - link with tests to execute
 */

#include <iostream>

#include "gtest/gtest.h"

GTEST_API_ int main(int argc, char **argv) {
  std::cout << "Running main() from gtest_main.cpp\n";

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



