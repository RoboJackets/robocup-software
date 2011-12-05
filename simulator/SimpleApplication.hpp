/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:

 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btAlignedObjectArray.h"

class btCollisionShape;
class btDynamicsWorld;
class btRigidBody;
class btTypedConstraint;

class SimpleCamera;

class SimpleApplication {
protected:
	// World dynamics model components
	btClock m_clock;
	btDynamicsWorld* m_dynamicsWorld;
	bool m_stepping;
	bool m_singleStep;
	int m_lastKey;
	int m_debugMode;

	// Camera components
	SimpleCamera* _camera;

	// GUI components
public:
	int m_modifierKeys;
protected:

	btScalar m_defaultContactProcessingThreshold;

public:

	SimpleApplication();

	virtual ~SimpleApplication();

	btDynamicsWorld* getDynamicsWorld() {
		return m_dynamicsWorld;
	}

	virtual void initPhysics() = 0;

	virtual void setDrawClusters(bool drawClusters) {}

	int getDebugMode() const {
		return m_debugMode;
	}

	SimpleCamera* camera() { return _camera; }

	void setDebugMode(int mode);

	btScalar getDeltaTimeMicroseconds() {
		btScalar dt = (btScalar) m_clock.getTimeMicroseconds();
		m_clock.reset();
		return dt;
	}

	///glut callbacks

	void moveAndDisplay();

	virtual void clientMoveAndDisplay() = 0;

	virtual void clientResetScene();

	btRigidBody* localCreateRigidBody(float mass,
			const btTransform& startTransform, btCollisionShape* shape);

	///callback methods by glut	

	virtual void keyboardCallback(unsigned char key, int x, int y);

	virtual void keyboardUpCallback(unsigned char key, int x, int y) {}

	virtual void specialKeyboard(int key, int x, int y);

	virtual void specialKeyboardUp(int key, int x, int y) {}

	virtual void updateModifierKeys();

};

