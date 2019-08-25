#include <optional>

#include <protobuf/LogFrame.pb.h>
#include <Geometry2d/Line.hpp>
#include <Geometry2d/Polygon.hpp>
#include <LogUtils.hpp>
#include <Robot.hpp>
#include <RobotConfig.hpp>
#include <SystemState.hpp>
#include "DebugDrawer.hpp"
#include "planning/Path.hpp"

using namespace Packet;
using namespace std;
using namespace Planning;
using namespace Geometry2d;
using Planning::MotionInstant;

class BallPath : public Planning::Path {
public:
    BallPath(const Ball& ball) : ball(ball){};

    virtual bool hit(const Geometry2d::ShapeSet& obstacles,
                     RJ::Seconds startTimeIntoPath,
                     RJ::Seconds* hitTime) const {
        throw new std::runtime_error("Unsupported Opperation");
    }

    virtual void draw(DebugDrawer* constdebug_drawer,
                      const QColor& color = Qt::black,
                      const QString& layer = "Motion") const {
        throw new std::runtime_error("Unsupported Opperation");
    }

    virtual RJ::Seconds getDuration() const { return RJ::Seconds::max(); }

    virtual std::unique_ptr<Path> subPath(RJ::Seconds startTime,
                                          RJ::Seconds endTime) const {
        throw new std::runtime_error("Unsupported Opperation");
    }

    virtual RobotInstant start() const {
        return RobotInstant(ball.predict(startTime()));
    }
    virtual RobotInstant end() const {
        throw new std::runtime_error("Unsupported Opperation");
    }

    virtual std::unique_ptr<Path> clone() const {
        return std::make_unique<BallPath>(*this);
    }

protected:
    virtual std::optional<RobotInstant> eval(RJ::Seconds t) const {
        return RobotInstant(ball.predict(startTime() + t));
    }

private:
    const Ball& ball;
};

std::unique_ptr<Planning::Path> Ball::path(RJ::Time startTime) const {
    auto path = std::make_unique<BallPath>(*this);
    path->setStartTime(startTime);
    return std::move(path);
}

constexpr auto ballDecayConstant = 0.180;

Planning::MotionInstant Ball::predict(RJ::Time estimateTime) const {
    if (estimateTime < time) {
        // debugThrow("Estimated Time can't be before observation time.");
        std::cout
            << "CRITICAL ERROR: Estimated Time can't be before observation time"
            << std::endl;
        std::cout << "estimateTime: " << RJ::timestamp(estimateTime)
                  << std::endl;
        std::cout << "actualTime: " << RJ::timestamp(time) << std::endl;
        estimateTime = time;

        // return MotionInstant();
    }

    MotionInstant instant;
    auto t = RJ::Seconds(estimateTime - time);

    const auto s0 = vel.mag();

    double speed = 0;
    double distance = 0;
    if (s0 != 0) {
        auto maxTime = s0 / ballDecayConstant;
        if (t.count() >= maxTime) {
            speed = 0;
            distance = s0 * maxTime - pow(maxTime, 2) / 2.0 * ballDecayConstant;
        } else {
            speed = s0 - (t.count() * ballDecayConstant);
            distance = s0 * t.count() - pow(t.count(), 2) / 2.0 * ballDecayConstant;
        }
    } else {
        speed = 0;
        distance = 0;
    }

    // Based on sim ball
    // v = v0 * e^-0.2913t
    // d = v0 * -3.43289 (-1 + e^(-0.2913 t))
    // auto part = std::exp(-0.2913f * t.count());
    // auto speed = s0 * part;
    // auto distance = s0 * -3.43289f * (part - 1.0f);

    return MotionInstant(pos + vel.normalized(distance), vel.normalized(speed));
}

Geometry2d::Point Ball::predictPosition(double seconds_from_now) const {
    const auto motionInstant = this->predict(RJ::now() + RJ::Seconds(seconds_from_now));
    return motionInstant.pos;
}

RJ::Time Ball::estimateTimeTo(const Geometry2d::Point& point,
                              Geometry2d::Point* nearPointOut) const {
    Line line(pos, pos + vel);
    auto nearPoint = line.nearestPoint(point);
    if (nearPointOut) {
        *nearPointOut = nearPoint;
    }
    auto dist = nearPoint.distTo(pos);
    // d = v0t - 1/2*t^2*Constant
    // d = v0t - 1/2*t^2*Constant
    // t = (v - sqrt(-2 C d + v^2))/C

    auto v = vel.mag();
    auto part = pow(v, 2) - 2 * ballDecayConstant * dist;
    if (part > 0) {
        auto t = (v - sqrt(part)) / ballDecayConstant;
        return time + RJ::Seconds(t);
    } else {
        return RJ::Time::max();
    }

    // auto part = vel.mag() * -3.43289;
}

double Ball::estimateSecondsTo(const Geometry2d::Point &point) const {
    const auto time = estimateTimeTo(point);
    return RJ::Seconds(time - RJ::now()).count();
}

double Ball::predictSecondsToStop() const {
    return vel.mag()/ballDecayConstant;
}

double Ball::estimateSecondsToDist(double dist) const {
    auto v = vel.mag();
    auto part = pow(v, 2) - 2 * ballDecayConstant * dist;
    auto t = (vel.mag() - part) / ballDecayConstant;
    if (part > 0) {
        auto t = (v - sqrt(part)) / ballDecayConstant;
        return t;
    } else {
        return std::numeric_limits<double>::infinity();
    }
}

SystemState::SystemState(Context* const context) {
    // FIXME - boost::array?
    self.resize(Num_Shells);
    opp.resize(Num_Shells);
    for (unsigned int i = 0; i < Num_Shells; ++i) {
        self[i] = new OurRobot(i, context);
        opp[i] = new OpponentRobot(i);
    }
}

SystemState::~SystemState() {
    for (unsigned int i = 0; i < Num_Shells; ++i) {
        delete self[i];
        delete opp[i];
    }
}

std::vector<int> SystemState::ourValidIds() {
    std::vector<int> validIds;
    for (int i = 0; i < self.size(); i++) {
        if (self[i]->visible) {
            validIds.push_back(self[i]->shell());
        }
    }
    return validIds;
}
