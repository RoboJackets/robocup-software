#include "MotionControl.hpp"

#include <optional>

#include <Geometry2d/Util.hpp>
#include <Robot.hpp>
#include <RobotConfig.hpp>
#include <SystemState.hpp>
#include <Utils.hpp>
#include <planning/MotionInstant.hpp>
#include "DebugDrawer.hpp"
#include "TrapezoidalMotion.hpp"

#include <stdio.h>
#include <algorithm>

using namespace std;
using namespace Geometry2d;
using namespace Planning;

#pragma mark Config Variables

REGISTER_CONFIGURABLE(MotionControl);

ConfigDouble* MotionControl::_max_acceleration;
ConfigDouble* MotionControl::_max_velocity;
ConfigDouble* MotionControl::_x_multiplier;

void MotionControl::createConfiguration(Configuration* cfg) {
    _max_acceleration =
        new ConfigDouble(cfg, "MotionControl/Max Acceleration", 1.5);
    _max_velocity = new ConfigDouble(cfg, "MotionControl/Max Velocity", 2.0);
    _x_multiplier = new ConfigDouble(cfg, "MotionControl/X_Multiplier", 1.0);
}

#pragma mark MotionControl

MotionControl::MotionControl(Context* context, OurRobot* robot)
    : _angleController(0, 0, 0, 50, 0), _context(context) {
    _robot = robot;

    _robot->robotPacket.set_uid(_robot->shell());
}

void MotionControl::run() {
    if (!_robot) return;

    const MotionConstraints& constraints = _robot->motionConstraints();

    // update PID parameters
    _positionXController.kp = *_robot->config->translation.p;
    _positionXController.ki = *_robot->config->translation.i;
    _positionXController.setWindup(*_robot->config->translation.i_windup);
    _positionXController.kd = *_robot->config->translation.d;
    _positionYController.kp = *_robot->config->translation.p;
    _positionYController.ki = *_robot->config->translation.i;
    _positionYController.setWindup(*_robot->config->translation.i_windup);
    _positionYController.kd = *_robot->config->translation.d;
    _angleController.kp = *_robot->config->rotation.p;
    _angleController.ki = *_robot->config->rotation.i;
    _angleController.kd = *_robot->config->rotation.d;

    RJ::Seconds timeIntoPath =
        (RJ::now() - _robot->path().startTime()) + RJ::Seconds(1.0 / 60);

    // evaluate path - where should we be right now?
    std::optional<RobotInstant> optTarget =
        _robot->path().evaluate(timeIntoPath);

    if (!optTarget) {
        optTarget = _robot->path().end();
        _context->debug_drawer.drawCircle(optTarget->motion.pos, .15, Qt::red,
                                          "Planning");
    } else {
        Point start = _robot->pos;
        _context->debug_drawer.drawCircle(optTarget->motion.pos, .15, Qt::green,
                                          "Planning");
    }

    // Angle control //////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////

    float targetW = 0;
    auto& rotationCommand = _robot->rotationCommand();
    const auto& rotationConstraints = _robot->rotationConstraints();

    std::optional<Geometry2d::Point> targetPt;
    const auto& motionCommand = _robot->motionCommand();

    std::optional<float> targetAngleFinal;
    // if (motionCommand->getCommandType() == MotionCommand::Pivot) {
    //    PivotCommand command =
    //    *static_cast<PivotCommand*>(motionCommand.get());
    //    targetPt = command.pivotTarget;
    //} else {
    if (optTarget) {
        if (optTarget->angle) {
            if (optTarget->angle->angle) {
                targetAngleFinal = *optTarget->angle->angle;
            }
        }
    }
    //}

    if (targetPt) {
        // fixing the angle ensures that we don't go the long way around to get
        // to our final angle
        targetAngleFinal = (*targetPt - _robot->pos).angle();
    }

    if (!targetAngleFinal) {
        _targetAngleVel(0);
    } else {
        float angleError = fixAngleRadians(*targetAngleFinal - _robot->angle);

        targetW = _angleController.run(angleError);

        // limit W
        if (abs(targetW) > (rotationConstraints.maxSpeed)) {
            if (targetW > 0) {
                targetW = (rotationConstraints.maxSpeed);
            } else {
                targetW = -(rotationConstraints.maxSpeed);
            }
        }

        /*
        _robot->addText(QString("targetW: %1").arg(targetW));
        _robot->addText(QString("angleError: %1").arg(angleError));
        _robot->addText(QString("targetGlobalAngle: %1").arg(targetAngleFinal));
        _robot->addText(QString("angle: %1").arg(_robot->angle));
        */
        _targetAngleVel(targetW);
    }

    // handle body velocity for pivot command
    /*
    if (motionCommand->getCommandType() == MotionCommand::Pivot) {
        float r = Robot_Radius;
        const float FudgeFactor = *_robot->config->pivotVelMultiplier;
        float speed = RadiansToDegrees(r * targetW * FudgeFactor);
        Point vel(speed, 0);

        // the robot body coordinate system is wierd...
        vel.rotate(-M_PI_2);

        _targetBodyVel(vel);

        return;  // pivot handles both angle and position
    }
     */

    // Position control ///////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    MotionInstant target = optTarget->motion;

    // tracking error
    Point posError = target.pos - _robot->pos;

    // acceleration factor
    Point acceleration;
    std::optional<RobotInstant> nextTarget =
        _robot->path().evaluate(timeIntoPath + RJ::Seconds(1) / 60.0);
    if (nextTarget) {
        acceleration = (nextTarget->motion.vel - target.vel) / 60.0f;
    } else {
        acceleration = {0, 0};
    }
    Point accelFactor =
        acceleration * 60.0f * (*_robot->config->accelerationMultiplier);

    target.vel += accelFactor;

    // PID on position
    target.vel.x() += _positionXController.run(posError.x());
    target.vel.y() += _positionYController.run(posError.y());

    // draw target pt
    _context->debug_drawer.drawCircle(target.pos, .04, Qt::red,
                                      "MotionControl");
    _context->debug_drawer.drawLine(target.pos, target.pos + target.vel,
                                    Qt::blue, "MotionControl");

    // Clamp World Acceleration
    auto dt = RJ::Seconds(RJ::now() - _lastCmdTime);
    Point targetAccel = (target.vel - _lastWorldVelCmd) / dt.count();
    targetAccel.clamp(*_max_acceleration);

    target.vel = _lastWorldVelCmd + targetAccel * dt.count();

    _lastWorldVelCmd = target.vel;
    _lastCmdTime = RJ::now();

    // convert from world to body coordinates
    // the +y axis of the robot points forwards
    target.vel = target.vel.rotated(M_PI_2 - _robot->angle);

    this->_targetBodyVel(target.vel);
}

void MotionControl::stopped() {
    _targetBodyVel(Point(0, 0));
    _targetAngleVel(0);
}

void MotionControl::_targetAngleVel(float angleVel) {
    // velocity multiplier
    angleVel *= *_robot->config->angleVelMultiplier;

    // If the angular speed is very low, it won't make the robot move at all, so
    // we make sure it's above a threshold value
    float minEffectiveAngularSpeed = *_robot->config->minEffectiveAngularSpeed;
    if (std::abs(angleVel) < minEffectiveAngularSpeed &&
        std::abs(angleVel) > .05) {
        angleVel =
            angleVel > 0 ? minEffectiveAngularSpeed : -minEffectiveAngularSpeed;
    }

    // the robot firmware still speaks degrees, so that's how we send it over
    _robot->control->set_avelocity(angleVel);
}

void MotionControl::_targetBodyVel(Point targetVel) {
    // Limit Velocity
    targetVel.clamp(*_max_velocity);

    // Limit Acceleration

    // make sure we don't send any bad values
    if (std::isnan(targetVel.x()) || std::isnan(targetVel.y())) {
        targetVel = Point(0, 0);
        debugThrow("A bad value was calculated.");
    }

    // track these values so we can limit acceleration

    // velocity multiplier
    targetVel *= *_robot->config->velMultiplier;

    // if the velocity is nonzero, make sure it's not so small that the robot
    // doesn't even move
    float minEffectiveVelocity = *_robot->config->minEffectiveVelocity;
    if (targetVel.mag() < minEffectiveVelocity && targetVel.mag() > 0.02) {
        targetVel = targetVel.normalized() * minEffectiveVelocity;
    }

    // set control values
    _robot->control->set_xvelocity(targetVel.x() * _x_multiplier->value());
    _robot->control->set_yvelocity(targetVel.y());
}

Pid* MotionControl::getPid(char controller) {
    // just in case there is confusion
    switch (tolower(controller)) {
        case 'a':
            return &_angleController;
        case 'x':
            return &_positionXController;
        case 'y':
            return &_positionYController;
        default:
            return &_positionXController;
    }
}
