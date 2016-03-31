#pragma once
#include "Entity.hpp"
#include "SimEngine.hpp"
#include "RaycastVehicle.hpp"

#include <protobuf/RadioTx.pb.h>
#include <protobuf/Control.pb.h>
#include <protobuf/RadioRx.pb.h>

class Ball;
class RobotBallController;
class GL_ShapeDrawer;

class Robot : public Entity {
public:
    enum WheelIndex {
        FrontLeft = 0,
        FrontRight = 1,
        BackRight = 2,
        BackLeft = 3
    };

    typedef enum { rev2008, rev2011 } RobotRevision;

    // assigned shell number
    unsigned int shell;

    int visibility;

protected:
    RobotRevision _rev;

    // physics components
    btRigidBody* _robotChassis;
    RaycastVehicle::btVehicleTuning _tuning;
    btVehicleRaycaster* _vehicleRayCaster;
    RaycastVehicle* _robotVehicle;
    btCollisionShape* _wheelShape;

    RobotBallController* _controller;

    // control inputs
    float _engineForce[4];
    float _brakingForce;

    btVector3 _targetVel;
    btScalar _targetRot;

    // state info
    btTransform _startTransform;

    // links to the engine
    SimEngine* _simEngine;

    /** center of roller from ground */
    constexpr static float RollerHeight = .03;
    /** center of roller from center of robot */
    constexpr static float RollerOffset = .065;
    /** roller length */
    constexpr static float RollerLength = .07;
    /** radius of the roller */
    constexpr static float RollerRadius = .01;

    /** width of the kicker face */
    constexpr static float KickerFaceWidth = .05;
    /** height of the kicker face */
    constexpr static float KickerFaceHeight = .005;
    /** depth of the kicker plate */
    constexpr static float KickerLength = .03;

public:
    Robot(Environment* env, unsigned int id, Robot::RobotRevision rev,
          Geometry2d::Point startPos);
    ~Robot();

    void initPhysics(const bool& blue);

    // the following are in field space, aka soccer's coordinate system

    // Entity interface
    virtual void position(float x, float y) override;  // world coords

    virtual void velocity(float x, float y, float w);  // body coords

    virtual Geometry2d::Point getPosition() const override;

    virtual float getAngle() const;

    // debug text interface
    Geometry2d::Point getVelFS() const;

    float getAngVelFS() const;

    Geometry2d::Point getTargetVelFS() const;

    float getTargetAngVelFS() const;

public:
    const RobotRevision& revision() const { return _rev; }

    const float* getEngineForce() const { return _engineForce; }

    float getBrakingForce() const { return _brakingForce; }

    btRigidBody* getRigidBody() const { return _robotChassis; }

    RaycastVehicle* getRaycastVehicle() const { return _robotVehicle; }

    RobotBallController* getRobotController() const { return _controller; }

    SimEngine* getSimEngine() { return _simEngine; }

    void getWorldTransform(btTransform& chassisWorldTrans) const;

    void setEngineForce(int wheelIndex, float val) {
        _engineForce[wheelIndex] = val;
    }

    void setEngineForce(float val) {
        for (int i = 0; i < 4; i++) {
            _engineForce[i] = val;
        }
    }

    void setBrakingForce(float val) { _brakingForce = val; }

    /** set control data */
    void radioTx(const Packet::Control* data);

    /** get robot information data */
    Packet::RadioRx radioRx() const;

    void renderWheels(GL_ShapeDrawer* shapeDrawer,
                      const btVector3& worldBoundsMin,
                      const btVector3& worldBoundsMax) const;

    void applyEngineForces();

    void applyEngineForces(float deltaTime);

    void resetScene();

    // bang-bang steering and throttle
    void steerLeft();
    void steerRight();
    void driveForward();
    void driveBackward();
};
