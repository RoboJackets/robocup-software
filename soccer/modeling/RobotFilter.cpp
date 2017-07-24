#include "RobotFilter.hpp"
#include <Utils.hpp>
#include <iostream>

using namespace std;
using namespace Geometry2d;

// How long to coast a robot's position when it isn't visible
static const RJ::Seconds Coast_Time(0.8);
static const float Velocity_Alpha = 0.2;
static const RJ::Seconds Min_Frame_Time(0.014);
static const RJ::Seconds Min_Velocity_Valid_Time(0.1);
static const RJ::Seconds Vision_Timeout_Time(0.25);
static const RJ::Seconds Min_Double_Packet_Time(1.0/120);

RobotFilter::RobotFilter() {}

void RobotFilter::update(const std::array<RobotObservation, Num_Cameras> &observations, RobotPose* robot, RJ::Time currentTime, u_int32_t frameNumber) {
    bool anyValid = std::any_of(observations.begin(), observations.end(), [](const RobotObservation& obs) {return obs.valid;});
    if (anyValid) {
        for (int i=0; i<observations.size(); i++) {
            const auto& obs = observations[i];
            auto& estimate = _estimates[i];
            if (obs.valid) {
                Point velEstimate{};
                double angleVelEstimate = 0;
                const auto dtime = RJ::Seconds(obs.time - estimate.time);
                if (dtime < Min_Double_Packet_Time) {
                    velEstimate = estimate.vel;
                    angleVelEstimate = estimate.angleVel;
                    estimate.velValid = true;
                } else if (dtime < Min_Velocity_Valid_Time) {
                    velEstimate = (obs.pos - estimate.pos)/dtime.count();
                    angleVelEstimate = fixAngleRadians(obs.angle - estimate.angle)/dtime.count();
                    estimate.velValid = true;
                } else if (robot->velValid) {
                    velEstimate = robot->vel;
                    angleVelEstimate = robot->angleVel;
                }

                
                if (dtime < Min_Velocity_Valid_Time && estimate.velValid) {
                    estimate.vel = velEstimate*Velocity_Alpha + estimate.vel * (1.0f - Velocity_Alpha);
                    estimate.angleVel = fixAngleRadians(angleVelEstimate*Velocity_Alpha + estimate.angleVel * (1.0f - Velocity_Alpha));
                } else {
                    estimate.vel = velEstimate;
                    estimate.angleVel = fixAngleRadians(angleVelEstimate);
                }

                estimate.pos = obs.pos;
                estimate.angle = obs.angle;
                estimate.visible = true;
                estimate.time = obs.time;
                estimate.visionFrame = obs.frameNumber;
            }
        }

        Point positionTotal{};
        double positionWeightTotal = 0;

        Point velocityTotal{};
        double velocityWeightTotal = 0;

        double angleTotal = 0;
        double angleWeightTotal = 0;

        double angleVelTotal = 0;
        double angleVelWeightTotal = 0;
        for (const auto &estimate: _estimates) {
            const auto dTime = RJ::Seconds(currentTime - estimate.time);
            debugThrowIf("dTime is less than 0", dTime < RJ::Seconds(0));
            if (estimate.visible && dTime < Vision_Timeout_Time) {
                Point pos{};
                double angle{};
                double currentPosWeight = std::max(0.0, 1.0-std::pow(dTime/Vision_Timeout_Time, 2));
                if (estimate.velValid) {
                    pos = estimate.pos + estimate.vel*dTime.count();
                    velocityTotal += estimate.vel*currentPosWeight;
                    velocityWeightTotal += currentPosWeight;

                    angle = estimate.angle + estimate.angleVel*dTime.count();
                    angleVelTotal += estimate.angleVel*currentPosWeight;
                    angleVelWeightTotal += currentPosWeight;
                } else {
                    pos = estimate.pos;

                    angle = estimate.angle;
                    currentPosWeight /= 2;
                }
                positionTotal += pos*currentPosWeight;
                positionWeightTotal += currentPosWeight;

                angleTotal += angle*currentPosWeight;
                angleWeightTotal += currentPosWeight;
            }
        }

        _currentEstimate.pos = positionTotal / positionWeightTotal;
        if (velocityWeightTotal > 0) {
            _currentEstimate.vel = velocityTotal / velocityWeightTotal;
            _currentEstimate.velValid = true;
        } else {
            _currentEstimate.vel = Point();
            _currentEstimate.velValid = false;
        }

        _currentEstimate.visible = true;
        _currentEstimate.time = currentTime;
        _currentEstimate.visionFrame = frameNumber;
        _currentEstimate.angle = angleTotal / angleWeightTotal;

        if (angleVelWeightTotal > 0) {
            _currentEstimate.angleVel = angleVelTotal / angleVelWeightTotal;
        } else {
            _currentEstimate.angleVel = 0;
        }
    }

    if (currentTime - _currentEstimate.time < Vision_Timeout_Time) {
        *robot = _currentEstimate;
    } else {
        robot->visible = false;
    }
}
//
//void RobotFilter::update(const RobotObservation* obs) {
//    if (obs->source < 0 || obs->source >= Num_Cameras) {
//        // Not from a camera?
//        return;
//    }
//
//    int s = obs->source;
//    RJ::Seconds dtime = (obs->time - _estimates[s].time);
//    // bool reset = _currentEstimate[s].time == 0 || (dtime > Coast_Time);
//
//    bool reset = (dtime > Coast_Time);
//    if (reset) {
////        _estimate[s].vel = Point();
////        _estimate[s].angleVel = 0;
//    } else {
//        Point newVel = (obs->pos - _estimates[s].pos) / dtime.count();
//        _estimates[s].vel = newVel * Velocity_Alpha +
//                           _estimates[s].vel * (1.0f - Velocity_Alpha);
//
//        double newW =
//            fixAngleRadians(obs->angle - _estimates[s].angle) / dtime.count();
//        _estimates[s].angleVel = newW * Velocity_Alpha +
//                                _estimates[s].angleVel * (1.0f - Velocity_Alpha);
//    }
//    _estimates[s].pos = obs->pos;
//    _estimates[s].angle = obs->angle;
//    _estimates[s].visible = true;
//    _estimates[s].time = obs->time;
//    _estimates[s].visionFrame = obs->frameNumber;
//
//    _currentEstimate = obs->pos;
//    _currentEstimate.angle = obs->angle;
//}
//
//void RobotFilter::predict(RJ::Time time, RobotPose* robot) {
//    int bestSource = -1;
//    RJ::Seconds bestDTime = RJ::Seconds::min();
//    for (int s = 0; s < Num_Cameras; ++s) {
//        RJ::Seconds dtime = (time - _estimates[s].time);
//        if (_estimates[s].visible && (bestSource < 0 || dtime < bestDTime)) {
//            bestSource = s;
//            bestDTime = dtime;
//        }
//    }
//
//    if (bestSource < 0) {
//        robot->visible = false;
//        return;
//    }
//
//    robot->pos = _estimates[bestSource].pos +
//                 _estimates[bestSource].vel * bestDTime.count();
//    robot->vel = _estimates[bestSource].vel;
//    robot->angle =
//        fixAngleRadians(_estimates[bestSource].angle +
//                        _estimates[bestSource].angleVel * bestDTime.count());
//    robot->angleVel = _estimates[bestSource].angleVel;
//    robot->visible = _estimates[bestSource].visible && bestDTime < Coast_Time;
//}
