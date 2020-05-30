#include "Trajectory.hpp"

namespace Trajectory {

int Trajectory::getNumEvals(double duration, double granularity) {
    auto iterations = std::ceil(duration / granularity) + 1;
    return static_cast<int>(iterations);
}

std::unique_ptr<InterpolatedPath> Trajectory::rasterizePath(
    std::unique_ptr<Path>&& path, const std::optional<AngleFn>& angle_fn,
    double granularity) {
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
        const bool final_iteration = i == iterations - 1;
        const double time = final_iteration ? path_duration : i * granularity;

        const RJ::Seconds t{time};

        std::optional<RobotInstant> maybe_instant = path->evaluate(t);

        if (maybe_instant) {
            // Append the angles to InterpolatedPath
            if (angle_fn) {
                const auto angle = (*angle_fn)(maybe_instant->motion);
                maybe_instant->angle = angle;
            }

            entries.emplace_back(*maybe_instant, t);
        }
    }

    const auto& start_time = path->startTime();

    return std::make_unique<InterpolatedPath>(std::move(entries), start_time);
}

}  // namespace Trajectory
