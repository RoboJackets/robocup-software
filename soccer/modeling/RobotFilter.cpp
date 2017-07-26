#include "RobotFilter.hpp"
#include <Utils.hpp>
#include <iostream>

using namespace std;
using namespace Geometry2d;

// How long to coast a robot's position when it isn't visible
static const RJ::Seconds Coast_Time(0.8);
static const RJ::Seconds Min_Frame_Time(0.014);
static const RJ::Seconds Min_Velocity_Valid_Time(0.1);
static const RJ::Seconds Vision_Timeout_Time(0.25);
static const RJ::Seconds Min_Double_Packet_Time(1.0/120);

REGISTER_CONFIGURABLE(RobotFilter);

ConfigDouble* RobotFilter::_velocity_alpha;

RobotFilter::RobotFilter() {}

void RobotFilter::createConfiguration(Configuration *cfg) {
    _velocity_alpha = new ConfigDouble(cfg, "RobotFilter/Velocity_Alpha", 0.2);
}

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

                const auto velocityAlpha = *_velocity_alpha;
                if (dtime < Min_Velocity_Valid_Time && estimate.velValid) {
                    estimate.vel = velEstimate*velocityAlpha + estimate.vel * (1.0f - velocityAlpha);
                    estimate.angleVel = fixAngleRadians(angleVelEstimate*velocityAlpha + estimate.angleVel * (1.0f - velocityAlpha));
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
//            debugLogIf(to_string(dTime) + " dTime is less than 0", dTime < RJ::Seconds(0));
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
