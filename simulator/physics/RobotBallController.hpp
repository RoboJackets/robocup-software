#pragma once

#include <BulletDynamics/Dynamics/btActionInterface.h>
#include "SimEngine.hpp"
#include <Utils.hpp>

#include <stdint.h>

class Robot;
class Ball;
class btPairCachingGhostObject;

/// RobotBallController is an object that handles dribbling and kicking/chipping
/// for the parent robot.
/// It uses a ghost object to test for collisions with the ball.
class RobotBallController : public btActionInterface {
public:
    bool ballSensorWorks;
    bool chargerWorks;

protected:
    btPairCachingGhostObject* _ghostObject;

    btVector3 _localMouthPos;

    Robot* _parent;
    btRigidBody* _ball;

    /// links to the engine
    SimEngine* _simEngine;

    /// kicker charge status
    uint64_t _lastKicked;
    const static uint64_t RechargeTime = 6000000;  // six seconds

    float _kickSpeed;

    uint64_t _kick;
    bool _chip;
    uint64_t _dribble;

public:
    RobotBallController(Robot* robot);
    ~RobotBallController();

    void initPhysics();

    /// btActionInterface interface
    virtual void updateAction(btCollisionWorld* collisionWorld,
                              btScalar deltaTime) override {
        detectBall(collisionWorld);
        dribblerStep();
        kickerStep();
    }

    /// btActionInterface interface
    void debugDraw(btIDebugDraw* debugDrawer) override{};

    bool detectBall(btCollisionWorld* collisionWorld);
    void dribblerStep();
    void kickerStep();

    bool hasBall();
    bool getKickerStatus();

    uint64_t getKickPower() { return _kick; }
    bool chipEnabled() { return _chip; }
    uint64_t getDribblePower() { return _dribble; }

    void syncMotionState(const btTransform& centerOfMassWorldTrans);

    const uint64_t getLastKickTime() { return _lastKicked; }

    /**
     * @brief Prepare to kick the ball as soon as one is detected by the "ball
     * sensor"
     * @param power A value from 0 to 255 indicating what proportion of the max
     * kick power to apply.  A value of zero "unprimes" the kicker.
     * @param chip A boolean indicating whether or not to use the chipper.  If
     * false, just uses the regular kicker.
     */
    void prepareKick(uint64_t power, bool chip);
    void prepareDribbler(uint64_t dribble);
};
