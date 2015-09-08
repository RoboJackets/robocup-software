#include "MotionControl.hpp"
#include <SystemState.hpp>
#include <RobotConfig.hpp>
#include <Robot.hpp>
#include <Utils.hpp>
#include "TrapezoidalMotion.hpp"
#include <Geometry2d/Util.hpp>
#include <planning/MotionInstant.hpp>

#include <cmath>
#include <stdio.h>
#include <algorithm>

using namespace std;
using namespace Geometry2d;
using namespace Planning;

#pragma mark Config Variables

REGISTER_CONFIGURABLE(MotionControl);

ConfigDouble* MotionControl::_max_acceleration;
ConfigDouble* MotionControl::_max_velocity;

void MotionControl::createConfiguration(Configuration* cfg) {
    _max_acceleration =
        new ConfigDouble(cfg, "MotionControl/Max Acceleration", 1.5);
    _max_velocity = new ConfigDouble(cfg, "MotionControl/Max Velocity", 2.0);
}

#pragma mark MotionControl

MotionControl::MotionControl(OurRobot* robot) : _angleController(0, 0, 0, 50) {
    _robot = robot;

    _robot->radioTx.set_robot_id(_robot->shell());
    _lastCmdTime = -1;
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

    // Angle control //////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////

    float targetW = 0;
    if (constraints.targetAngleVel) {
        targetW = *constraints.targetAngleVel;
    } else if (constraints.faceTarget || constraints.pivotTarget) {
        Point targetPt = constraints.pivotTarget ? *constraints.pivotTarget
                                                 : *constraints.faceTarget;

        // fixing the angle ensures that we don't go the long way around to get
        // to our final angle
        float targetAngleFinal = (targetPt - _robot->pos).angle();
        float angleError = fixAngleRadians(targetAngleFinal - _robot->angle);

        // float targetW;
        // float targetAngle;
        // TrapezoidalMotion(
        // 	abs(angleError),					// dist
        // 	motionConstraints.maxAngleSpeed,	// max deg/sec
        // 	30,									// max
        // deg/sec^2
        // 	0.1 ,								// time into
        // path
        // 	_robot->angleVel,					// start speed
        // 	0,									// final
        // speed
        // 	targetAngle,
        // 	targetW 							//
        // ignored
        // 	);

        // // PID on angle
        // if(angleError<0) {
        // 	targetW = - targetW;
        // }
        // targetW = _angleController.run(targetAngle);

        targetW = _angleController.run(angleError);

        // limit W
        if (abs(targetW) > (constraints.maxAngleSpeed)) {
            if (targetW > 0) {
                targetW = (constraints.maxAngleSpeed);
            } else {
                targetW = -(constraints.maxAngleSpeed);
            }
        }

        /*
        _robot->addText(QString("targetW: %1").arg(targetW));
        _robot->addText(QString("angleError: %1").arg(angleError));
        _robot->addText(QString("targetGlobalAngle: %1").arg(targetAngleFinal));
        _robot->addText(QString("angle: %1").arg(_robot->angle));
        */
    }

    _targetAngleVel(targetW);

    // handle body velocity for pivot command
    if (constraints.pivotTarget) {
        float r = Robot_Radius;
        const float FudgeFactor = *_robot->config->pivotVelMultiplier;
        float speed = r * targetW * RadiansToDegrees * FudgeFactor;
        Point vel(speed, 0);

        // the robot body coordinate system is wierd...
        vel.rotate(-M_PI_2);

        _targetBodyVel(vel);

        return;  // pivot handles both angle and position
    }

    // Position control ///////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////

    MotionInstant target;

    // if no target position is given, we don't have a path to follow
    if (!_robot->path()) {
        _targetBodyVel(Point(0, 0));
        return;
    } else {
        //
        // Path following
        //

        // convert from microseconds to seconds
        float timeIntoPath =
            ((float)(timestamp() - _robot->path()->startTime())) *
                TimestampToSecs +
            1.0 / 60.0;

        // evaluate path - where should we be right now?
        boost::optional<MotionInstant> optTarget =
            _robot->path()->evaluate(timeIntoPath);
        if (!optTarget) {
            target.vel = Point();
            target.pos = _robot->pos;
        } else {
            target = *optTarget;
        }
        // tracking error
        Point posError = target.pos - _robot->pos;

        // acceleration factor
        Point acceleration;
        boost::optional<MotionInstant> nextTarget =
            _robot->path()->evaluate(timeIntoPath + 1.0 / 60.0);
        if (nextTarget) {
            acceleration = (nextTarget->vel - target.vel) / 60.0f;
        } else {
            acceleration = {0, 0};
        }
        Point accelFactor =
            acceleration * 60.0f * (*_robot->config->accelerationMultiplier);

        target.vel += accelFactor;

        // PID on position
        target.vel.x += _positionXController.run(posError.x);
        target.vel.y += _positionYController.run(posError.y);

        // draw target pt
        _robot->state()->drawCircle(target.pos, .04, Qt::red, "MotionControl");
        _robot->state()->drawLine(target.pos, target.pos + target.vel, Qt::blue,
                                  "MotionControl");

        // convert from world to body coordinates
        target.vel = target.vel.rotated(-_robot->angle);
    }

    this->_targetBodyVel(target.vel);
}

void MotionControl::stopped() {
    _targetBodyVel(Point(0, 0));
    _targetAngleVel(0);
}

void MotionControl::_targetAngleVel(float angleVel) {
    // velocity multiplier
    angleVel *= *_robot->config->angleVelMultiplier;

    // convert units
    angleVel *= RadiansToDegrees;

    // If the angular speed is very low, it won't make the robot move at all, so
    // we make sure it's above a threshold value
    float minEffectiveAngularSpeed = *_robot->config->minEffectiveAngularSpeed;
    if (std::abs(angleVel) < minEffectiveAngularSpeed &&
        std::abs(angleVel) > 0.2) {
        angleVel =
            angleVel > 0 ? minEffectiveAngularSpeed : -minEffectiveAngularSpeed;
    }

    // the robot firmware still speaks degrees, so that's how we send it over
    _robot->radioTx.set_body_w(angleVel);
}

void MotionControl::_targetBodyVel(Point targetVel) {
    // Limit Velocity
    targetVel.clamp(*_max_velocity);

    // Limit Acceleration
    if (_lastCmdTime == -1) {
        targetVel.clamp(*_max_acceleration);
    } else {
        float dt = (float)((timestamp() - _lastCmdTime) / 1000000.0f);
        Point targetAccel = (targetVel - _lastVelCmd) / dt;
        targetAccel.clamp(*_max_acceleration);

        targetVel = _lastVelCmd + targetAccel * dt;
    }

    // make sure we don't send any bad values
    if (isnan(targetVel.x) || isnan(targetVel.y)) {
        targetVel = Point(0, 0);
    }

    // track these values so we can limit acceleration
    _lastVelCmd = targetVel;
    _lastCmdTime = timestamp();

    // velocity multiplier
    targetVel *= *_robot->config->velMultiplier;

    // if the velocity is nonzero, make sure it's not so small that the robot
    // doesn't even move
    float minEffectiveVelocity = *_robot->config->minEffectiveVelocity;
    if (targetVel.mag() < minEffectiveVelocity && targetVel.mag() > 0.05) {
        targetVel = targetVel.normalized() * minEffectiveVelocity;
    }

    // set radioTx values
    _robot->radioTx.set_body_x(targetVel.x);
    _robot->radioTx.set_body_y(targetVel.y);
}
