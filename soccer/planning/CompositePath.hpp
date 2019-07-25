#pragma once

#include <optional>

#include <planning/Path.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include <Configuration.hpp>

namespace Planning {

/**
 * @brief Represents a motion path made up of a series of Paths.
 *
 * @details The path represents a function of position given time that the robot
 *     should follow. The path is made up of other Paths and can be made up of
 *     CompositePaths.
 */
class CompositePath : public Path {
private:
    // Vector of Paths
    std::vector<std::unique_ptr<Path>> paths;

    // Saving some variables to speed up computation
    RJ::Seconds duration = 0ms;

public:
    /** default path is empty */
    CompositePath() {}

    /** constructors with one path */
    CompositePath(std::unique_ptr<Path> path);

    template <typename... Args>
    CompositePath(std::unique_ptr<Path> path, Args... args)
        : CompositePath(std::move(path)) {
        append(std::forward<Args>(args)...);
    }

    /**
     * Append the path to the end of the CompositePath
     */
    void append(std::unique_ptr<Path> path);

    template <typename... Args>
    void append(std::unique_ptr<Path> path, Args... args) {
        append(std::move(path));
        append(std::forward<Args>(args)...);
    }

    virtual bool hit(const Geometry2d::ShapeSet& shape,
                     RJ::Seconds startTimeIntoPath,
                     RJ::Seconds* hitTime) const override;
    virtual void draw(DebugDrawer* constdebug_drawer,
                      const QColor& color = Qt::black,
                      const QString& layer = "Motion") const override;
    virtual RJ::Seconds getDuration() const override;
    virtual std::unique_ptr<Path> subPath(
        RJ::Seconds startTime = 0ms,
        RJ::Seconds endTime = RJ::Seconds::max()) const override;
    virtual RobotInstant start() const override;
    virtual RobotInstant end() const override;
    virtual std::unique_ptr<Path> clone() const override;

protected:
    virtual std::optional<RobotInstant> eval(RJ::Seconds t) const override;
};

}  // namespace Planning
