#pragma once

#include <Geometry2d/Point.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include <SystemState.hpp>
#include "MotionInstant.hpp"

#include <boost/optional.hpp>
#include <QColor>
#include <QString>

namespace Planning {

/**
 * @brief Abstract class representing a motion path
 */
class Path {
public:
    Path(RJ::Time startTime = RJ::timestamp()) : _startTime(startTime) {}
    virtual ~Path() {}

    /**
     * This method evaluates the path at a given time and returns the target
     * angle,
     * position, and velocity of the robot.
     *
     * @param t Time (in seconds) since the robot started the path. Throws an
     *     exception if t<0
     * @return A RobotInstant containing the angle, position, and velocity at
     * the given
     *     time if @t is within the range of the path.  If @t is not within the
     *     time range of this path, this method returns boost::none.
     */
    virtual boost::optional<RobotInstant> evaluate(float t) const = 0;

    /**
     * Returns true if the path hits an obstacle
     *
     * @param[in]	shape The obstacles on the field
     * @param[out]  hitTime the approximate time when the path hits an obstacle.
     * If no obstacles are hit, behavior is undefined for the final value.
     * @param[in] 	startTime The time on the path to start checking from
     * @return 		true if it hits an obstacle, otherwise false
     */
    virtual bool hit(const Geometry2d::ShapeSet& obstacles, float& hitTime,
                     float startTime) const = 0;

    /**
     * Draws the path.  The default implementation adds a DebugRobotPath to the
     * SystemState that interpolates points along the path.
     *
     * @param state The SystemState to draw the path on
     * @param color The color the path should be drawn
     * @param layer The layer to draw the path on
     */
    virtual void draw(SystemState* const state, const QColor& color = Qt::black,
                      const QString& layer = "Motion") const;

    /**
     * Returns how long it would take for the entire path to be traversed
     *
     * @return The time from start to path completion or infinity if it never
     * stops
     */
    virtual float getDuration() const = 0;

    /**
     * Returns a subPath
     *
     * @param startTime The startTime for from which the subPath should be
     *     taken.
     * @param endTime The endTime from which the subPath should be taken. If it
     *     is greater than the duration fo the path, it should go to the end of
     *     the path.
     * @return A unique_ptr to the new subPath
     */
    virtual std::unique_ptr<Path> subPath(
        float startTime = 0,
        float endTime = std::numeric_limits<float>::infinity()) const = 0;

    /// Start instant of the path
    virtual RobotInstant start() const = 0;
    /// Destination instant of the path
    virtual RobotInstant end() const = 0;

    /**
     * Returns a deep copy of the Path
     */
    virtual std::unique_ptr<Path> clone() const = 0;

    /// The time the path starts at
    virtual RJ::Time startTime() const { return _startTime; }
    virtual void setStartTime(RJ::Time t) { _startTime = t; }

protected:
    RJ::Time _startTime;
};

/**
 * Class which represents a Path with an angle Function attached.
 */
class AngleFunctionPath : public Path {
public:
    AngleFunctionPath(
        std::unique_ptr<Path> path = nullptr,
        boost::optional<std::function<AngleInstant(MotionInstant)>>
            angleFunction = boost::none)
        : path(std::move(path)), angleFunction(angleFunction) {}

    std::unique_ptr<Path> path;
    boost::optional<std::function<AngleInstant(MotionInstant)>> angleFunction;
    /**
     * This method evaluates the path at a given time and returns the target
     * position and velocity of the robot.
     *
     * @param t Time (in seconds) since the robot started the path. Throws an
     *     exception if t<0
     * @return A MotionInstant containing the position and velocity at the given
     *     time if @t is within the range of the path.  If @t is not within the
     *     time range of this path, this method returns boost::none.
     */
    virtual boost::optional<RobotInstant> evaluate(float t) const override {
        if (!path) {
            return boost::none;
        }

        boost::optional<RobotInstant> instant = path->evaluate(t);
        if (!angleFunction) {
            return instant;
        } else {
            if (instant) {
                instant->angle = angleFunction->operator()(instant->motion);
                return instant;
            } else {
                return boost::none;
            }
        }
    }

    /**
     * Returns true if the path hits an obstacle
     *
     * @param[in]	shape The obstacles on the field
     * @param[out]  hitTime the approximate time when the path hits an obstacle.
     * If no obstacles are hit, behavior is undefined for the final value.
     * @param[in] 	startTime The time on the path to start checking from
     * @return 		true if it hits an obstacle, otherwise false
     */
    virtual bool hit(const Geometry2d::ShapeSet& obstacles, float& hitTime,
                     float startTime) const override {
        return path->hit(obstacles, hitTime, startTime);
    }

    /**
     * Draws the path.  The default implementation adds a DebugRobotPath to the
     * SystemState that interpolates points along the path.
     *
     * @param state The SystemState to draw the path on
     * @param color The color the path should be drawn
     * @param layer The layer to draw the path on
     */
    virtual void draw(SystemState* const state, const QColor& color = Qt::black,
                      const QString& layer = "Motion") const override {
        path->draw(state, color, layer);
    }

    /**
     * Returns how long it would take for the entire path to be traversed
     *
     * @return The time from start to path completion or infinity if it never
     * stops
     */
    virtual float getDuration() const override { return path->getDuration(); }

    /**
     * Returns a subPath
     *
     * @param startTime The startTime for from which the subPath should be
     *     taken.
     * @param endTime The endTime from which the subPath should be taken. If it
     *     is greater than the duration fo the path, it should go to the end of
     *     the path.
     * @return A unique_ptr to the new subPath
     */
    virtual std::unique_ptr<Path> subPath(
        float startTime = 0,
        float endTime = std::numeric_limits<float>::infinity()) const override {
        return std::make_unique<AngleFunctionPath>(
            path->subPath(startTime, endTime), angleFunction);
    }

    /// Start instant of the path
    virtual RobotInstant start() const override {
        RobotInstant instant = path->start();
        if (angleFunction) {
            instant.angle = angleFunction->operator()(instant.motion);
            return instant;
        }
        return instant;
    }
    /// Destination instant of the path
    virtual RobotInstant end() const override {
        RobotInstant instant = path->end();
        if (angleFunction) {
            instant.angle = angleFunction->operator()(instant.motion);
            return instant;
        }
        return instant;
    }

    /**
     * Returns a deep copy of the Path
     */
    virtual std::unique_ptr<Path> clone() const override {
        return std::make_unique<AngleFunctionPath>(path->clone(),
                                                   angleFunction);
    }

    virtual RJ::Time startTime() const override { return path->startTime(); }
    virtual void setStartTime(RJ::Time t) override { path->setStartTime(t); }
};

}  // namespace Planning
