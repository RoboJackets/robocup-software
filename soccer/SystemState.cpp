#include <optional>

#include <protobuf/LogFrame.pb.h>
#include <Geometry2d/Line.hpp>
#include <Geometry2d/Polygon.hpp>
#include <LogUtils.hpp>
#include <Robot.hpp>
#include <RobotConfig.hpp>
#include <SystemState.hpp>
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

    virtual void draw(SystemState* const state, const QColor& color = Qt::black,
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

constexpr auto ballDecayConstant = 0.275;

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
    return RJ::Seconds(RJ::now() - time).count();
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

SystemState::SystemState(Context* context) {
    _numDebugLayers = 0;

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

int SystemState::findDebugLayer(QString layer) {
    if (layer.isNull()) {
        layer = "Debug";
    }

    QMap<QString, int>::const_iterator i = _debugLayerMap.find(layer);
    if (i == _debugLayerMap.end()) {
        // New layer
        int n = _numDebugLayers++;
        _debugLayerMap[layer] = n;
        _debugLayers.append(layer);
        return n;
    } else {
        // Existing layer
        return i.value();
    }
}

void SystemState::drawPolygon(const Geometry2d::Point* pts, int n,
                              const QColor& qc, const QString& layer) {
    DebugPath* dbg = logFrame->add_debug_polygons();
    dbg->set_layer(findDebugLayer(layer));
    for (int i = 0; i < n; ++i) {
        *dbg->add_points() = pts[i];
    }
    dbg->set_color(color(qc));
}

void SystemState::drawPolygon(const std::vector<Geometry2d::Point>& pts,
                              const QColor& qc, const QString& layer) {
    drawPolygon(pts.data(), pts.size(), qc, layer);
}

void SystemState::drawPolygon(const Geometry2d::Polygon& polygon,
                              const QColor& qc, const QString& layer) {
    this->drawPolygon(polygon.vertices, qc, layer);
}

void SystemState::drawCircle(Geometry2d::Point center, float radius,
                             const QColor& qc, const QString& layer) {
    DebugCircle* dbg = logFrame->add_debug_circles();
    dbg->set_layer(findDebugLayer(layer));
    *dbg->mutable_center() = center;
    dbg->set_radius(radius);
    dbg->set_color(color(qc));
}

void SystemState::drawArc(const Geometry2d::Arc& arc, const QColor& qc,
                          const QString& layer) {
    DebugArc* dbg = logFrame->add_debug_arcs();
    dbg->set_layer(findDebugLayer(layer));
    *dbg->mutable_center() = arc.center();
    dbg->set_radius(arc.radius());
    dbg->set_start(arc.start());
    dbg->set_end(arc.end());
    dbg->set_color(color(qc));
}

void SystemState::drawShape(const std::shared_ptr<Geometry2d::Shape>& obs,
                            const QColor& color, const QString& layer) {
    std::shared_ptr<Geometry2d::Circle> circObs =
        std::dynamic_pointer_cast<Geometry2d::Circle>(obs);
    std::shared_ptr<Geometry2d::Polygon> polyObs =
        std::dynamic_pointer_cast<Geometry2d::Polygon>(obs);
    std::shared_ptr<Geometry2d::CompositeShape> compObs =
        std::dynamic_pointer_cast<Geometry2d::CompositeShape>(obs);
    if (circObs)
        drawCircle(circObs->center, circObs->radius(), color, layer);
    else if (polyObs)
        drawPolygon(polyObs->vertices, color, layer);
    else if (compObs) {
        for (const std::shared_ptr<Geometry2d::Shape>& obs :
             compObs->subshapes())
            drawShape(obs, color, layer);
    }
}

void SystemState::drawShapeSet(const Geometry2d::ShapeSet& shapes,
                               const QColor& color, const QString& layer) {
    for (auto& shape : shapes.shapes()) {
        drawShape(shape, color, layer);
    }
}

void SystemState::drawLine(const Geometry2d::Segment& line, const QColor& qc,
                           const QString& layer) {
    DebugPath* dbg = logFrame->add_debug_paths();
    dbg->set_layer(findDebugLayer(layer));
    *dbg->add_points() = line.pt[0];
    *dbg->add_points() = line.pt[1];
    dbg->set_color(color(qc));
}

void SystemState::drawLine(Geometry2d::Point p0, Geometry2d::Point p1,
                           const QColor& color, const QString& layer) {
    drawLine(Geometry2d::Segment(p0, p1), color, layer);
}

void SystemState::drawText(const QString& text, Geometry2d::Point pos,
                           const QColor& qc, const QString& layer) {
    DebugText* dbg = logFrame->add_debug_texts();
    dbg->set_layer(findDebugLayer(layer));
    dbg->set_text(text.toStdString());
    *dbg->mutable_pos() = pos;
    dbg->set_color(color(qc));
}

void SystemState::drawSegment(const Geometry2d::Segment& line, const QColor& qc,
                              const QString& layer) {
    DebugPath* dbg = logFrame->add_debug_paths();
    dbg->set_layer(findDebugLayer(layer));
    *dbg->add_points() = line.pt[0];
    *dbg->add_points() = line.pt[1];
    dbg->set_color(color(qc));
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
