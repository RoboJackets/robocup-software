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

#include "GlutDefs.hpp"
#include "GL_ShapeDrawer.h"

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

class GlutCamera {
protected:
	float m_cameraDistance;
	float m_ele;
	float m_azi;
	btVector3 m_cameraPosition;
	btVector3 m_cameraTargetPosition; //look at
	float m_scaleBottom;
	float m_scaleFactor;
	btVector3 m_cameraUp;
	int m_forwardAxis;
	int m_glutScreenWidth;
	int m_glutScreenHeight;
	float m_frustumZNear;
	float m_frustumZFar;
	int m_ortho;
	GL_ShapeDrawer* m_shapeDrawer;
	bool m_enableshadows;
	btVector3 m_sundirection;

	// connect to dynamics
	btDynamicsWorld* m_dynamicsWorld;

public:
	GlutCamera(btDynamicsWorld* world = 0);
	~GlutCamera();

	void setDynamicsWorld(btDynamicsWorld* world) { m_dynamicsWorld = world; }
	btDynamicsWorld* getDynamicsWorld() { return m_dynamicsWorld; }

	bool setTexturing(bool enable) { return (m_shapeDrawer->enableTexture(enable)); }
	bool setShadows(bool enable) {
		bool p = m_enableshadows;
		m_enableshadows = enable;
		return (p);
	}

	bool getTexturing() const { return m_shapeDrawer->hasTextureEnabled(); }
	bool getShadows() const {	return m_enableshadows; }

	void setAzi(float azi) { m_azi = azi; }

	void setCameraUp(const btVector3& camUp) { m_cameraUp = camUp; }
	void setCameraForwardAxis(int axis) {	m_forwardAxis = axis;	}

	void setCameraPosition(const btVector3& pos) { m_cameraPosition = pos; }
	btVector3 getCameraPosition() const { return m_cameraPosition; }

	void setCameraTargetPosition(const btVector3& pos) { m_cameraTargetPosition = pos; }
	btVector3 getCameraTargetPosition() const {	return m_cameraTargetPosition; }

	GL_ShapeDrawer* shapeDrawer() { return m_shapeDrawer; }

	//	void overrideGLShapeDrawer(GL_ShapeDrawer* shapeDrawer); // FIXME: necessary?

	void setFrustumZPlanes(float zNear, float zFar) {
		m_frustumZNear = zNear;
		m_frustumZFar = zFar;
	}

	virtual void myinit(); // initializes camera

	// performs camera updates - can be customized for different camera modes
	virtual void updateCamera();

	void setCameraDistance(float dist) { m_cameraDistance = dist; }
	float getCameraDistance() const {	return m_cameraDistance; }

	void renderscene(int pass, int debugMode);

	virtual void reshape(int w, int h);   /// Rerender due to resizing the window

	virtual void displayCallback() {}

	virtual void renderme(int debugMode);

	virtual void swapBuffers();

	btVector3 getRayTo(int x, int y);

	void displayProfileString(int xOffset, int yStart, char* message);

	// allow for drawing to screen without projection
	void setOrthographicProjection();
	void resetPerspectiveProjection();

	void chaseCamera();

}; // \class GlutCamera

