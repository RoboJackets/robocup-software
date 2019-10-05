#pragma once

#include <optional>

#include <Geometry2d/Point.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include "MotionInstant.hpp"
#include "Utils.hpp"

#include <DebugDrawer.hpp>
#include <QColor>
#include <QString>

#include "DynamicObstacle.hpp"

class SystemState;
namespace Planning {

class ConstPathIterator;
/**
 * @brief Abstract class representing a motion path
 */
class Path {
public:
    Path(RJ::Time startTime = RJ::now()) : _startTime(startTime) {}
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
     *     time range of this path, this method returns std::nullopt
     */
    std::optional<RobotInstant> evaluate(RJ::Seconds t) const {
        auto instant = eval(t * evalRate);
        if (instant) {
            instant->motion.vel *= evalRate;
        }
        return instant;
    }

    /**
     * Returns true if the path hits an obstacle
     *
     * @param[in]	shape The obstacles on the field
     * @param[out]  hitTime the approximate time when the path hits an obstacle.
     * If no obstacles are hit, behavior is undefined for the final value.
     * @param[in] 	startTimeIntoPath The time on the path to start checking
     *from
     * @return 		true if it hits an obstacle, otherwise false
     */
    virtual bool hit(const Geometry2d::ShapeSet& obstacles,
                     RJ::Seconds startTimeIntoPath,
                     RJ::Seconds* hitTime = nullptr) const = 0;

    /**
     * Draws the path.  The default implementation adds a DebugRobotPath to the
     * SystemState that interpolates points along the path.
     *
     * @param debug_drawer The SystemState to draw the path on
     * @param color The color the path should be drawn
     * @param layer The layer to draw the path on
     */
    virtual void draw(DebugDrawer* const debug_drawer,
                      const QColor& color = Qt::black,
                      const QString& layer = "Motion") const;

    /**
     * Returns how long it would take for the entire path to be traversed
     *
     * @return The time from start to path completion or infinity if it never
     * stops
     */
    virtual RJ::Seconds getDuration() const = 0;

    RJ::Seconds getSlowedDuration() const { return getDuration() / evalRate; }

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
        RJ::Seconds startTime = 0ms,
        RJ::Seconds endTime = RJ::Seconds::max()) const = 0;

    /// Start instant of the path
    virtual RobotInstant start() const = 0;
    /// Destination instant of the path
    virtual RobotInstant end() const = 0;

    /**
     * Returns a deep copy of the Path
     */
    virtual std::unique_ptr<Path> clone() const = 0;

    virtual void setDebugText(QString string) {
        _debugText = std::move(string);
    }

    virtual void drawDebugText(DebugDrawer* debug_drawer,
                               const QColor& color = Qt::darkCyan,
                               const QString& layer = "PathDebugText") const;

    /// The time the path starts at
    virtual RJ::Time startTime() const { return _startTime; }
    virtual void setStartTime(RJ::Time t) { _startTime = t; }

    virtual bool pathsIntersect(const std::vector<DynamicObstacle>& paths,
                                RJ::Time startTime,
                                Geometry2d::Point* hitLocation,
                                RJ::Seconds* hitTime) const;

    virtual std::unique_ptr<ConstPathIterator> iterator(
        RJ::Time startTime, RJ::Seconds deltaT) const;

    void slow(float multiplier, RJ::Seconds timeInto = RJ::Seconds::zero());

protected:
    virtual std::optional<RobotInstant> eval(RJ::Seconds t) const = 0;

    double evalRate = 1.0;
    RJ::Time _startTime;
    std::optional<QString> _debugText;
};

/**
 * Class which represents a Path with an angle Function attached.
 */
class AngleFunctionPath : public Path {
public:
    AngleFunctionPath(
        std::unique_ptr<Path> path = nullptr,
        std::optional<std::function<AngleInstant(MotionInstant)>>
            angleFunction = std::nullopt)
        : path(std::move(path)), angleFunction(angleFunction) {}

    std::unique_ptr<Path> path;
    std::optional<std::function<AngleInstant(MotionInstant)>> angleFunction;

    /**
     * Returns true if the path hits an obstacle
     *
     * @param[in]	shape The obstacles on the field
     * @param[out]  hitTime the approximate time when the path hits an obstacle.
     * If no obstacles are hit, behavior is undefined for the final value.
     * @param[in] 	startTimeIntoPath The time on the path to start checking
     *from
     * @return 		true if it hits an obstacle, otherwise false
     */
    virtual bool hit(const Geometry2d::ShapeSet& obstacles,
                     RJ::Seconds startTimeIntoPath,
                     RJ::Seconds* hitTime) const override {
        return path->hit(obstacles, startTimeIntoPath, hitTime);
    }

    /**
     * Draws the path.  The default implementation adds a DebugRobotPath to the
     * SystemState that interpolates points along the path.
     *
     * @param debug_drawer The SystemState to draw the path on
     * @param color The color the path should be drawn
     * @param layer The layer to draw the path on
     */
    virtual void draw(DebugDrawer* const debug_drawer,
                      const QColor& color = Qt::black,
                      const QString& layer = "Motion") const override {
        path->draw(debug_drawer, color, layer);
    }

    /**
     * Returns how long it would take for the entire path to be traversed
     *
     * @return The time from start to path completion or infinity if it never
     * stops
     */
    virtual RJ::Seconds getDuration() const override {
        return path->getDuration();
    }

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
        RJ::Seconds startTime = RJ::Seconds::zero(),
        RJ::Seconds endTime = RJ::Seconds::max()) const override {
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
    virtual void setDebugText(QString string) override {
        path->setDebugText(std::move(string));
    }

    virtual void drawDebugText(
        DebugDrawer* debug_drawer, const QColor& color = Qt::darkCyan,
        const QString& layer = "PathDebugText") const override {
        path->drawDebugText(debug_drawer, color, layer);
    }

protected:
    /**
     * This method evaluates the path at a given time and returns the target
     * position and velocity of the robot.
     *
     * @param t Time (in seconds) since the robot started the path. Throws an
     *     exception if t<0
     * @return A MotionInstant containing the position and velocity at the given
     *     time if @t is within the range of the path.  If @t is not within the
     *     time range of this path, this method returns std::nullopt.
     */
    virtual std::optional<RobotInstant> eval(RJ::Seconds t) const override {
        if (!path) {
            return std::nullopt;
        }

        std::optional<RobotInstant> instant = path->evaluate(t);
        if (!angleFunction) {
            return instant;
        } else {
            if (instant) {
                instant->angle = angleFunction->operator()(instant->motion);
                return instant;
            } else {
                return std::nullopt;
            }
        }
    }
};

class ConstPathIterator {
public:
    explicit ConstPathIterator(const Path* path, RJ::Time startTime,
                               RJ::Seconds deltaT)
        : path(path), time(startTime - path->startTime()), deltaT(deltaT) {}

    virtual RobotInstant operator*() const {
        auto temp = path->evaluate(time);
        if (temp) {
            return *temp;
        } else {
            return path->end();
        }
    }

    virtual ConstPathIterator& operator++() {
        time += deltaT;
        return *this;
    }

private:
    const Path* const path;
    RJ::Seconds time;
    const RJ::Seconds deltaT;
};

}  // namespace Planning
