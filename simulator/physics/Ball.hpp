#pragma once

#include "Entity.hpp"
#include <physics/SimEngine.hpp>


class Ball: public Entity
{
protected:
	// physics components
	btRigidBody* _ball;
	btCollisionShape* _ballShape;

	// link to SimEngine
	SimEngine* _simEngine;

public:
	Ball(Environment* env);
	virtual ~Ball();

	virtual void position(float x, float y) override;
	virtual void velocity(float x, float y);

	virtual Geometry2d::Point getPosition() const override;

	void initPhysics();

	void resetScene();
};
