#pragma once

#include "Ball.hpp"

#include <physics/SimEngine.hpp>
#include <physics/PhysicsConstants.hpp>
#include "GL_ShapeDrawer.h"


class SimBall : public Ball
{
protected:
	// physics components
	btRigidBody* _ball;
	btCollisionShape* _ballShape;

	// link to SimEngine
	SimEngine* _simEngine;

public:
	SimBall(Environment* env);

	~SimBall() {
		delete _ballShape;
	}

	//Entity interface
	virtual void position(float x, float y);
	virtual void velocity(float x, float y);

	virtual Geometry2d::Point getPosition() const;

	void initPhysics();

	void resetScene();

};
