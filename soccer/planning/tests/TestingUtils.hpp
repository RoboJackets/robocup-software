#pragma once

namespace Planning::TestingUtils {
    /**
     * test if a path is continuous and well behaved
     * @param path
     * @param constraints
     */
    void assertPathContinuous(const Trajectory& path, const RobotConstraints& constraints);

    /**
     * generate a random number between lo and hi
     * @param lo
     * @param hi
     * @return random number
     */
    double random(double lo, double hi);

    /**
     * generate a random RobotInstant inside the field
     * @return random robot instant
     */
    RobotInstant randomInstant();
}