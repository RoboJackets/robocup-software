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

#include <iostream>
#include <stdexcept>

#include "GlutCamera.hpp"

#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btDefaultMotionState.h"

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "GLDebugFont.h"

////////////////////////////////////
// GlutCamera class
////////////////////////////////////

using namespace std;

GlutCamera::GlutCamera(SimEngine* engine) :
		m_cameraDistance(15.0), m_ele(20.f), m_azi(0.f), m_cameraPosition(0.f, 0.f, 0.f),
		m_cameraTargetPosition(0.f, 0.f, 0.f), m_scaleBottom(0.5f), m_scaleFactor(2.f),
		m_cameraUp(0, 1, 0), m_forwardAxis(2), m_glutScreenWidth(0), m_glutScreenHeight(0),
		m_frustumZNear(1.f), m_frustumZFar(10000.f), m_ortho(0), _simEngine(engine)
{
	m_shapeDrawer = new GL_ShapeDrawer();
	m_shapeDrawer->enableTexture(true);
	m_enableshadows = false;
}

GlutCamera::~GlutCamera() {
	if (m_shapeDrawer)
		delete m_shapeDrawer;
}

void GlutCamera::reshape(int w, int h) {
	GLDebugResetFont(w, h);

	m_glutScreenWidth = w;
	m_glutScreenHeight = h;

	glViewport(0, 0, w, h);
	updateCamera();
}

void GlutCamera::myinit(void) {

	GLfloat light_ambient[] = { btScalar(0.2), btScalar(0.2), btScalar(0.2),
			btScalar(1.0) };
	GLfloat light_diffuse[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0),
			btScalar(1.0) };
	GLfloat light_specular[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0),
			btScalar(1.0) };
	/*	light_position is NOT default value	*/
	GLfloat light_position0[] = { btScalar(1.0), btScalar(10.0), btScalar(1.0),
			btScalar(0.0) };
	GLfloat light_position1[] = { btScalar(-1.0), btScalar(-10.0), btScalar(-1.0),
			btScalar(0.0) };

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);

	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glClearColor(btScalar(0.7), btScalar(0.7), btScalar(0.7), btScalar(0));

	//  glEnable(GL_CULL_FACE);
	//  glCullFace(GL_BACK);
}

void GlutCamera::updateCamera() {

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	btScalar rele = m_ele * btScalar(0.01745329251994329547); // rads per deg
	btScalar razi = m_azi * btScalar(0.01745329251994329547); // rads per deg

	btQuaternion rot(m_cameraUp, razi);

	btVector3 eyePos(0, 0, 0);
	eyePos[m_forwardAxis] = -m_cameraDistance;

	btVector3 forward(eyePos[0], eyePos[1], eyePos[2]);
	if (forward.length2() < SIMD_EPSILON) {
		forward.setValue(1.f, 0.f, 0.f);
	}
	btVector3 right = m_cameraUp.cross(forward);
	btQuaternion roll(right, -rele);

	eyePos = btMatrix3x3(rot) * btMatrix3x3(roll) * eyePos;

	m_cameraPosition[0] = eyePos.getX();
	m_cameraPosition[1] = eyePos.getY();
	m_cameraPosition[2] = eyePos.getZ();
	m_cameraPosition += m_cameraTargetPosition;

	if (m_glutScreenWidth == 0 && m_glutScreenHeight == 0)
		return;

	btScalar aspect;
	btVector3 extents;

	aspect = m_glutScreenWidth / (btScalar) m_glutScreenHeight;
	extents.setValue(aspect * 1.0f, 1.0f, 0);

	if (m_ortho) {
		// reset matrix
		glLoadIdentity();

		extents *= m_cameraDistance;
		btVector3 lower = m_cameraTargetPosition - extents;
		btVector3 upper = m_cameraTargetPosition + extents;
		//gluOrtho2D(lower.x, upper.x, lower.y, upper.y);
		glOrtho(lower.getX(), upper.getX(), lower.getY(), upper.getY(), -1000,
				1000);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		//glTranslatef(100,210,0);
	} else {
//		glFrustum (-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);
		glFrustum(-aspect * m_frustumZNear, aspect * m_frustumZNear,
				-m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2],
				m_cameraTargetPosition[0], m_cameraTargetPosition[1],
				m_cameraTargetPosition[2], m_cameraUp.getX(), m_cameraUp.getY(),
				m_cameraUp.getZ());
	}

}

btVector3 GlutCamera::getRayTo(int x, int y) {

	if (m_ortho) {

		btScalar aspect;
		btVector3 extents;
		aspect = m_glutScreenWidth / (btScalar) m_glutScreenHeight;
		extents.setValue(aspect * 1.0f, 1.0f, 0);

		extents *= m_cameraDistance;
		btVector3 lower = m_cameraTargetPosition - extents;
		btVector3 upper = m_cameraTargetPosition + extents;

		btScalar u = x / btScalar(m_glutScreenWidth);
		btScalar v = (m_glutScreenHeight - y) / btScalar(m_glutScreenHeight);

		btVector3 p(0, 0, 0);
		p.setValue((1.0f - u) * lower.getX() + u * upper.getX(),
				(1.0f - v) * lower.getY() + v * upper.getY(),
				m_cameraTargetPosition.getZ());
		return p;
	}

	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top - bottom) * 0.5f / nearPlane;
	float fov = btScalar(2.0) * btAtan(tanFov);

	btVector3 rayFrom = getCameraPosition();
	btVector3 rayForward = (getCameraTargetPosition() - getCameraPosition());
	rayForward.normalize();
	float farPlane = 10000.f;
	rayForward *= farPlane;

	btVector3 rightOffset;
	btVector3 vertical = m_cameraUp;

	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f * fov);

	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	btScalar aspect;

	aspect = m_glutScreenWidth / (btScalar) m_glutScreenHeight;

	hor *= aspect;

	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f / float(m_glutScreenWidth);
	btVector3 dVert = vertical * 1.f / float(m_glutScreenHeight);

	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += btScalar(x) * dHor;
	rayTo -= btScalar(y) * dVert;
	return rayTo;
}


void GlutCamera::swapBuffers() {
	glutSwapBuffers();
}

//See http://www.lighthouse3d.com/opengl/glut/index.php?bmpfontortho
void GlutCamera::setOrthographicProjection() {

	// switch to projection mode
	glMatrixMode(GL_PROJECTION);

	// save previous matrix which contains the
	//settings for the perspective projection
	glPushMatrix();
	// reset matrix
	glLoadIdentity();
	// set a 2D orthographic projection
	gluOrtho2D(0, m_glutScreenWidth, 0, m_glutScreenHeight);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// invert the y axis, down is positive
	glScalef(1, -1, 1);
	// mover the origin from the bottom left corner
	// to the upper left corner
	glTranslatef(btScalar(0), btScalar(-m_glutScreenHeight), btScalar(0));

}

void GlutCamera::resetPerspectiveProjection() {
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	updateCamera();
}

void GlutCamera::displayProfileString(int xOffset, int yStart, char* message) {
	glRasterPos3f(btScalar(xOffset), btScalar(yStart), btScalar(0));
	GLDebugDrawString(xOffset, yStart, message);
}

void GlutCamera::renderscene(int pass, int debugMode) {
//	throw runtime_error("Breaking at renderscene");
	btScalar m[16];
	btMatrix3x3 rot;
	rot.setIdentity();
	const int numObjects = _simEngine->_dynamicsWorld->getNumCollisionObjects();
	btVector3 wireColor(1, 0, 0);
	for (int i = 0; i < numObjects; i++) {
		btCollisionObject* colObj = _simEngine->_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body && body->getMotionState()) {
			btDefaultMotionState* myMotionState =
					(btDefaultMotionState*) body->getMotionState();
			myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
			rot = myMotionState->m_graphicsWorldTrans.getBasis();
		} else {
			colObj->getWorldTransform().getOpenGLMatrix(m);
			rot = colObj->getWorldTransform().getBasis();
		}
		btVector3 wireColor(1.f, 1.0f, 0.5f); //wants deactivation
		if (i & 1)
			wireColor = btVector3(0.f, 0.0f, 1.f);
		///color differently for active, sleeping, wantsdeactivation states
		if (colObj->getActivationState() == 1) //active
				{
			if (i & 1) {
				wireColor += btVector3(1.f, 0.f, 0.f);
			} else {
				wireColor += btVector3(.5f, 0.f, 0.f);
			}
		}
		if (colObj->getActivationState() == 2) //ISLAND_SLEEPING
				{
			if (i & 1) {
				wireColor += btVector3(0.f, 1.f, 0.f);
			} else {
				wireColor += btVector3(0.f, 0.5f, 0.f);
			}
		}

		btVector3 aabbMin, aabbMax;
		_simEngine->_dynamicsWorld->getBroadphase()->getBroadphaseAabb(aabbMin, aabbMax);

		aabbMin -= btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
		aabbMax += btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
//		printf("aabbMin=(%f,%f,%f)\n",aabbMin.getX(),aabbMin.getY(),aabbMin.getZ());
//		printf("aabbMax=(%f,%f,%f)\n",aabbMax.getX(),aabbMax.getY(),aabbMax.getZ());
//		_dynamicsWorld->getDebugDrawer()->drawAabb(aabbMin,aabbMax,btVector3(1,1,1));

		if (!(debugMode & btIDebugDraw::DBG_DrawWireframe)) {
			switch (pass) {
			case 0:
				m_shapeDrawer->drawOpenGL(m, colObj->getCollisionShape(), wireColor,
						debugMode, aabbMin, aabbMax);
				break;
			case 1:
				m_shapeDrawer->drawShadow(m, m_sundirection * rot,
						colObj->getCollisionShape(), aabbMin, aabbMax);
				break;
			case 2:
				m_shapeDrawer->drawOpenGL(m, colObj->getCollisionShape(),
						wireColor * btScalar(0.3), 0, aabbMin, aabbMax);
				break;
			}
		}
	}
}

void GlutCamera::renderme(int debugMode) {
	myinit();
	updateCamera();
	if (_simEngine->_dynamicsWorld) {
		if (m_enableshadows) {
			glClear(GL_STENCIL_BUFFER_BIT);
			glEnable(GL_CULL_FACE);
			renderscene(0, debugMode);

			glDisable(GL_LIGHTING);
			glDepthMask(GL_FALSE);
			glDepthFunc(GL_LEQUAL);
			glEnable(GL_STENCIL_TEST);
			glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
			glStencilFunc(GL_ALWAYS, 1, 0xFFFFFFFFL);
			glFrontFace(GL_CCW);
			glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
			renderscene(1, debugMode);
			glFrontFace(GL_CW);
			glStencilOp(GL_KEEP, GL_KEEP, GL_DECR);
			renderscene(1, debugMode);
			glFrontFace(GL_CCW);

			glPolygonMode(GL_FRONT, GL_FILL);
			glPolygonMode(GL_BACK, GL_FILL);
			glShadeModel(GL_SMOOTH);
			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_LESS);
			glEnable(GL_LIGHTING);
			glDepthMask(GL_TRUE);
			glCullFace(GL_BACK);
			glFrontFace(GL_CCW);
			glEnable(GL_CULL_FACE);
			glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

			glDepthFunc(GL_LEQUAL);
			glStencilFunc(GL_NOTEQUAL, 0, 0xFFFFFFFFL);
			glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
			glDisable(GL_LIGHTING);
			renderscene(2, debugMode);
			glEnable(GL_LIGHTING);
			glDepthFunc(GL_LESS);
			glDisable(GL_STENCIL_TEST);
			glDisable(GL_CULL_FACE);
		} else {
			glDisable(GL_CULL_FACE);
			renderscene(0, debugMode);
		}
	}

	updateCamera();

}

void GlutCamera::chaseCamera() {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	btScalar aspect = m_glutScreenWidth / (btScalar) m_glutScreenHeight;
	glFrustum(-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2],
			m_cameraTargetPosition[0], m_cameraTargetPosition[1],
			m_cameraTargetPosition[2], m_cameraUp.getX(), m_cameraUp.getY(),
			m_cameraUp.getZ());
}
