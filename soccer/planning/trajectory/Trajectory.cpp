#include "Trajectory.hpp"

namespace Trajectory {

int Trajectory::getNumEvals(double duration, double granularity) {
    auto iterations = std::ceil(duration / granularity) + 1;
    return static_cast<int>(iterations);
}

std::unique_ptr<InterpolatedPath> Trajectory::rasterizePath(
    std::unique_ptr<Path>&& path, double granularity) {
    using Entry = Planning::InterpolatedPath::Entry;

    // First off, path shouldn't be nullptr
    assert(path != nullptr);  // NOLINT

    // Calculate the number of ticks to evaluate it at
    double path_duration = path->getDuration().count();
    int iterations = getNumEvals(path_duration, granularity);

    std::vector<Entry> entries;
    entries.reserve(iterations);

    // Evaluate it at a grid of points
    for (int i = 0; i < iterations; i++) {
        bool final_iteration = i == iterations - 1;
        double time = final_iteration ? path_duration : i * granularity;
        RJ::Seconds t{time};

        std::optional<RobotInstant> maybe_instant = path->evaluate(t);

        if (maybe_instant) {
            entries.emplace_back(*maybe_instant, t);
        }
    }

    return std::make_unique<InterpolatedPath>(std::move(entries));
}

}  // namespace Trajectory
