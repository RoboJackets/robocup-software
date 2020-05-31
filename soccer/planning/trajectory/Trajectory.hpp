#pragma once

#include <memory>
#include <planning/MotionInstant.hpp>
#include <planning/paths/InterpolatedPath.hpp>
#include <planning/paths/Path.hpp>
#include <utility>

namespace Trajectory {
using Planning::AngleInstant, Planning::MotionInstant, Planning::Path,
    Planning::InterpolatedPath, Planning::RobotInstant;

using AngleFn = std::function<AngleInstant(MotionInstant)>;

/**
 * \brief Represents a trajectory x(t), y(t), h(t) that is sent from the
 * PlannerNode to the MotionControlNode.
 *
 * TEMP: For simplicity, this is just a angleFunction attached to a
 * InterpolatedPath.
 */
class Trajectory {
public:
    /**
     * \brief Default constructor so we can make an array of this.
     */
    Trajectory() = default;

    /**
     * \brief Constructor that takes in any path and a angle function,
     * and "rasterizes" the path to be an InterpolatedPath
     *
     * @param path
     * @param angle_function
     * @param granularity The granularity in time with which to construct this
     * trajectory
     */
    Trajectory(std::unique_ptr<Path> path,
               const std::optional<AngleFn>& angle_function,
               RJ::Seconds granularity)
        : path_(rasterizePath(std::move(path), angle_function, granularity)) {}

    /**
     * @return Whether the trajectory contains a valid nonempty path
     */
    [[nodiscard]] inline bool hasPath() const {
        return path_ != nullptr && !path_->empty();
    }

    /**
     * @return The start time of the path
     */
    [[nodiscard]] inline RJ::Time startTime() const {
        assert(path_ != nullptr);  // NOLINT
        return path_->startTime();
    }

    /**
     * \brief Evaluates the trajectory at a given time and returns the target
     * RobotInstant.
     *
     * @param t
     * @return Target RobotInstant at passed in time
     */
    [[nodiscard]] inline std::optional<RobotInstant> evaluate(
        RJ::Seconds t) const {
        assert(path_ != nullptr);  // NOLINT

        RobotInstant instant;

        std::optional<RobotInstant> maybe_instant = path_->evaluate(t);

        if (maybe_instant) {
            instant = *maybe_instant;
        }
        return instant;
    }

    /**
     * \brief Returns how long it would take for the entire path to be traversed
     *
     * @return The time from start to path completion or infinity if it never
     * stops
     */
    [[nodiscard]] RJ::Seconds getDuration() const {
        assert(path_ != nullptr);  // NOLINT
        return path_->getDuration();
    }

    /**
     * @return Destination instant of the path
     */
    [[nodiscard]] RobotInstant end() const {
        assert(path_ != nullptr);  // NOLINT
        return path_->end();
    };

    /**
     * \brief Clears the trajectory.
     */
    void clear() { path_ = nullptr; }

    /**
     * \brief Moves the current path_ out, replacing it with nullptr.
     * @return The current path. This CAN BE nullptr, if it was nullptr before.
     */
    std::unique_ptr<InterpolatedPath> takePath() {
        std::unique_ptr<InterpolatedPath> current_path = std::move(path_);
        path_ = nullptr;
        return current_path;
    }

private:
    std::unique_ptr<InterpolatedPath> path_;

    /**
     * \brief Calculates the number of evaluations needed for the given
     * path duration and granularity.
     *
     * @param duration
     * @param granularity
     * @return
     */
    static int getNumEvals(double duration, double granularity);

    /**
     * \brief "Rasterizes" the passed in path by evaluating it at a grid of
     * points and constructing an InterpolatedPath from that.
     *
     * @param path
     * @param granularity The granularity of the "rasterized" path
     * @return The "rasterized" InterpolatedPath
     */
    static std::unique_ptr<InterpolatedPath> rasterizePath(
        std::unique_ptr<Path>&& path, const std::optional<AngleFn>& angle_fn,
        RJ::Seconds granularity);
};
}  // namespace Trajectory
