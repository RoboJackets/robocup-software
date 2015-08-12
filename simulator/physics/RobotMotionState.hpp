#pragma once

#include "LinearMath/btDefaultMotionState.h"
#include "RobotBallController.hpp"


///Synchronizes Robot and RobotBallController
struct RobotMotionState : public btDefaultMotionState
{
	RobotBallController* m_controller;

	RobotMotionState(const btTransform& startTrans = btTransform::getIdentity(),const btTransform& centerOfMassOffset = btTransform::getIdentity())
		: btDefaultMotionState(startTrans,centerOfMassOffset)
	{
	}

	///synchronizes world transform from user to physics
	virtual void	getWorldTransform(btTransform& centerOfMassWorldTrans ) const override
	{
			centerOfMassWorldTrans = 	m_centerOfMassOffset.inverse() * m_graphicsWorldTrans ;
	}

	///synchronizes world transform from physics to user
	///Bullet only calls the update of worldtransform for active objects
	virtual void	setWorldTransform(const btTransform& centerOfMassWorldTrans) override
	{
			m_graphicsWorldTrans = centerOfMassWorldTrans * m_centerOfMassOffset ;
			m_controller->syncMotionState(centerOfMassWorldTrans);
	}

};
