#pragma once

#include "Entity.hpp"
#include <GL/glut.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

class btTriangleIndexVertexArray;
class SimEngine;
class btVector3;

class Field : public Entity {
protected:
    // Terrain components
    class btTriangleIndexVertexArray* _indexVertexArrays;
    btVector3* _vertices;

    // links to the engine
    SimEngine* _simEngine;

public:
    Field(Environment* env);
    virtual ~Field();

    // Translates the origin of the field
    void position(float x, float y) override {
        _x = x;
        _y = y;
    }

    void initPhysics();

    void renderField();

    void reshapeBodies();

private:
    float _x;
    float _y;

    btRigidBody* _wallBodies[4];
    btRigidBody* _blueGoalBodies[3];
    btRigidBody* _yellowGoalBodies[3];

    // Utility functions for rendering
    void renderVerticalLine(float x1, float z1, float x2, float z2,
                            float height, float lineWidth);
    void renderHorizontalLine(float x1, float z1, float x2, float z2,
                              float height, float lineWidth);
    void renderArc(float x, float z, float angle1, float angle2, float height,
                   float radius, float lineWidth, int numPoints);
};
