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

#include <GLDebugDrawer.h>
#include "GlutDefs.hpp"
#include "GL_ShapeDrawer.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "Physics/SimEngine.hpp"

class btCollisionShape;
class btDynamicsWorld;
class btRigidBody;
class btTypedConstraint;

class GlutCamera {
protected:
	float _cameraDistance;
	float _ele;
	float _azi;
	btVector3 _cameraPosition;
	btVector3 _cameraTargetPosition; //look at
	float _scaleBottom;
	float _scaleFactor;
	btVector3 _cameraUp;
	int _forwardAxis;
	int _glutScreenWidth;
	int _glutScreenHeight;
	float _frustumZNear;
	float _frustumZFar;
	int _ortho;
	GL_ShapeDrawer* _shapeDrawer;
	bool _enableshadows;
	btVector3 _sundirection;

	GLDebugDrawer _debugDrawer;

	// connect to dynamics - not created internally
	SimEngine *_simEngine;

public:
	GlutCamera(SimEngine* engine = 0);
	~GlutCamera();

	bool setTexturing(bool enable) { return (_shapeDrawer->enableTexture(enable)); }
	bool setShadows(bool enable) {
		bool p = _enableshadows;
		_enableshadows = enable;
		return (p);
	}

	bool getTexturing() const { return _shapeDrawer->hasTextureEnabled(); }
	bool getShadows() const {	return _enableshadows; }

	void setAzi(float azi) { _azi = azi; }

	void setCameraUp(const btVector3& camUp) { _cameraUp = camUp; }
	void setCameraForwardAxis(int axis) {	_forwardAxis = axis;	}

	void setCameraPosition(const btVector3& pos) { _cameraPosition = pos; }
	btVector3 getCameraPosition() const { return _cameraPosition; }

	void setCameraTargetPosition(const btVector3& pos) { _cameraTargetPosition = pos; }
	btVector3 getCameraTargetPosition() const {	return _cameraTargetPosition; }

	GL_ShapeDrawer* shapeDrawer() { return _shapeDrawer; }
	GLDebugDrawer* debugDrawer() { return &_debugDrawer; }

	void setFrustumZPlanes(float zNear, float zFar) {
		_frustumZNear = zNear;
		_frustumZFar = zFar;
	}

	virtual void myinit(); // initializes camera

	// performs camera updates - can be customized for different camera modes
	virtual void updateCamera();

	void setCameraDistance(float dist) { _cameraDistance = dist; }
	float getCameraDistance() const {	return _cameraDistance; }

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

