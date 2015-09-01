#pragma once
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
    float duration = 0.0f;

public:
    /** default path is empty */
    CompositePath() {}

    /** constructors with one path */
    CompositePath(std::unique_ptr<Path> path);

    /**
     * Append the path to the end of the CompositePath
     */
    void append(std::unique_ptr<Path> path);

    virtual boost::optional<MotionInstant> evaluate(float t) const override;
    virtual bool hit(const Geometry2d::ShapeSet& shape, float& hitTime,
                     float startTime = 0) const override;
    virtual void draw(SystemState* const state, const QColor& color = Qt::black,
                      const QString& layer = "Motion") const override;
    virtual float getDuration() const override;
    virtual std::unique_ptr<Path> subPath(
        float startTime = 0,
        float endTime = std::numeric_limits<float>::infinity()) const override;
    virtual boost::optional<MotionInstant> destination() const override;
    virtual std::unique_ptr<Path> clone() const override;
    virtual bool valid() const override { return !paths.empty(); }
};

}  // namespace Planning
