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

#include "SimpleApplication.hpp"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"

#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"//picking
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"//picking
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btUniformScalingShape.h"
#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"
#include "GL_ShapeDrawer.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btSerializer.h"
#include "GLDebugFont.h"

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

//static bool use6Dof = false;
extern bool gDisableDeactivation;
int numObjects = 0;
const int maxNumObjects = 16384;
btTransform startTransforms[maxNumObjects];
btCollisionShape* gShapePtr[maxNumObjects]; //1 rigidbody has 1 shape (no re-use of shapes)
#define SHOW_NUM_DEEP_PENETRATIONS 1

//#define BT_NO_PROFILE

extern int gNumClampedCcdMotions;

#ifdef SHOW_NUM_DEEP_PENETRATIONS 
extern int gNumDeepPenetrationChecks;

extern int gNumGjkChecks;

#endif //
SimpleApplication::SimpleApplication()
//see btIDebugDraw.h for modes
:
		m_dynamicsWorld(0), m_shootBoxShape(0), m_cameraDistance(15.0), m_debugMode(
				0), m_ele(20.f), m_azi(0.f), m_cameraPosition(0.f, 0.f, 0.f), m_cameraTargetPosition(
				0.f, 0.f, 0.f), m_modifierKeys(0), m_scaleBottom(0.5f), m_scaleFactor(
				2.f), m_cameraUp(0, 1, 0), m_forwardAxis(2), m_zoomStepSize(0.4), m_glutScreenWidth(
				0), m_glutScreenHeight(0), m_frustumZNear(1.f), m_frustumZFar(10000.f), m_ortho(
				0), m_ShootBoxInitialSpeed(40.f), m_stepping(true), m_singleStep(false),
//				m_idle(false),

		m_enableshadows(false), m_sundirection(btVector3(1, -2, 1) * 1000), m_defaultContactProcessingThreshold(
				BT_LARGE_FLOAT) {
#ifndef BT_NO_PROFILE
	m_profileIterator = CProfileManager::Get_Iterator();
#endif //BT_NO_PROFILE
	m_shapeDrawer = new GL_ShapeDrawer();
	m_shapeDrawer->enableTexture(true);
	m_enableshadows = false;
}

SimpleApplication::~SimpleApplication() {
#ifndef BT_NO_PROFILE
	CProfileManager::Release_Iterator(m_profileIterator);
#endif //BT_NO_PROFILE
	if (m_shootBoxShape)
		delete m_shootBoxShape;

	if (m_shapeDrawer)
		delete m_shapeDrawer;
}

void SimpleApplication::overrideGLShapeDrawer(GL_ShapeDrawer* shapeDrawer) {
	shapeDrawer->enableTexture(m_shapeDrawer->hasTextureEnabled());
	delete m_shapeDrawer;
	m_shapeDrawer = shapeDrawer;
}

void SimpleApplication::myinit(void) {

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

void SimpleApplication::setCameraDistance(float dist) {
	m_cameraDistance = dist;
}

float SimpleApplication::getCameraDistance() {
	return m_cameraDistance;
}

void SimpleApplication::updateCamera() {

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

void SimpleApplication::reshape(int w, int h) {
	GLDebugResetFont(w, h);

	m_glutScreenWidth = w;
	m_glutScreenHeight = h;

	glViewport(0, 0, w, h);
	updateCamera();
}

void SimpleApplication::keyboardCallback(unsigned char key, int x, int y) {
	(void) x;
	(void) y;

	m_lastKey = 0;

#ifndef BT_NO_PROFILE
	if (key >= 0x31 && key <= 0x39) {
		int child = key - 0x31;
		m_profileIterator->Enter_Child(child);
	}
	if (key == 0x30) {
		m_profileIterator->Enter_Parent();
	}
#endif //BT_NO_PROFILE
	switch (key) {
	case 'q':
#ifdef BT_USE_FREEGLUT
		//return from glutMainLoop(), detect memory leaks etc.
		glutLeaveMainLoop();
#else
		exit(0);
#endif
		break;
	case 'g':
		m_enableshadows = !m_enableshadows;
		break;
	case 'u':
		m_shapeDrawer->enableTexture(!m_shapeDrawer->enableTexture(false));
		break;
	case 'h':
		if (m_debugMode & btIDebugDraw::DBG_NoHelpText)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_NoHelpText);
		else
			m_debugMode |= btIDebugDraw::DBG_NoHelpText;
		break;

	case 'w':
		if (m_debugMode & btIDebugDraw::DBG_DrawWireframe)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawWireframe);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawWireframe;
		break;

	case 'p':
		if (m_debugMode & btIDebugDraw::DBG_ProfileTimings)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_ProfileTimings);
		else
			m_debugMode |= btIDebugDraw::DBG_ProfileTimings;
		break;

	case '=': {
		int maxSerializeBufferSize = 1024 * 1024 * 5;
		btDefaultSerializer* serializer = new btDefaultSerializer(
				maxSerializeBufferSize);
		//serializer->setSerializationFlags(BT_SERIALIZE_NO_DUPLICATE_ASSERT);
		m_dynamicsWorld->serialize(serializer);
		FILE* f2 = fopen("testFile.bullet", "wb");
		fwrite(serializer->getBufferPointer(), serializer->getCurrentBufferSize(),
				1, f2);
		fclose(f2);
		delete serializer;
		break;

	}

	case 'm':
		if (m_debugMode & btIDebugDraw::DBG_EnableSatComparison)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_EnableSatComparison);
		else
			m_debugMode |= btIDebugDraw::DBG_EnableSatComparison;
		break;

	case 'n':
		if (m_debugMode & btIDebugDraw::DBG_DisableBulletLCP)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DisableBulletLCP);
		else
			m_debugMode |= btIDebugDraw::DBG_DisableBulletLCP;
		break;

	case 't':
		if (m_debugMode & btIDebugDraw::DBG_DrawText)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawText);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawText;
		break;
	case 'y':
		if (m_debugMode & btIDebugDraw::DBG_DrawFeaturesText)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawFeaturesText);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawFeaturesText;
		break;
	case 'a':
		if (m_debugMode & btIDebugDraw::DBG_DrawAabb)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawAabb);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawAabb;
		break;
	case 'c':
		if (m_debugMode & btIDebugDraw::DBG_DrawContactPoints)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawContactPoints);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawContactPoints;
		break;
	case 'C':
		if (m_debugMode & btIDebugDraw::DBG_DrawConstraints)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawConstraints);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawConstraints;
		break;
	case 'L':
		if (m_debugMode & btIDebugDraw::DBG_DrawConstraintLimits)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawConstraintLimits);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawConstraintLimits;
		break;

	case 'd':
		if (m_debugMode & btIDebugDraw::DBG_NoDeactivation)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_NoDeactivation);
		else
			m_debugMode |= btIDebugDraw::DBG_NoDeactivation;
		if (m_debugMode & btIDebugDraw::DBG_NoDeactivation) {
			gDisableDeactivation = true;
		} else {
			gDisableDeactivation = false;
		}
		break;

	case 'o': {
		m_ortho = !m_ortho; //m_stepping = !m_stepping;
		break;
	}
	case 's':
		clientMoveAndDisplay();
		break;
		//    case ' ' : newRandom(); break;
	case ' ':
		clientResetScene();
		break;
	case '1': {
		if (m_debugMode & btIDebugDraw::DBG_EnableCCD)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_EnableCCD);
		else
			m_debugMode |= btIDebugDraw::DBG_EnableCCD;
		break;
	}

	case '.': {
		shootBox(getRayTo(x, y)); //getCameraTargetPosition());
		break;
	}

	case '+': {
		m_ShootBoxInitialSpeed += 10.f;
		break;
	}
	case '-': {
		m_ShootBoxInitialSpeed -= 10.f;
		break;
	}

	default:
		//        std::cout << "unused key : " << key << std::endl;
		break;
	}

	if (getDynamicsWorld() && getDynamicsWorld()->getDebugDrawer())
		getDynamicsWorld()->getDebugDrawer()->setDebugMode(m_debugMode);

}

void SimpleApplication::setDebugMode(int mode) {
	m_debugMode = mode;
	if (getDynamicsWorld() && getDynamicsWorld()->getDebugDrawer())
		getDynamicsWorld()->getDebugDrawer()->setDebugMode(mode);
}

void SimpleApplication::moveAndDisplay() {
	clientMoveAndDisplay();
}

#define NUM_SPHERES_ON_DIAGONAL 9

void SimpleApplication::setShootBoxShape() {
	if (!m_shootBoxShape) {
		btBoxShape* box = new btBoxShape(btVector3(.5f, .5f, .5f));
		box->initializePolyhedralFeatures();
		m_shootBoxShape = box;
	}
}

void SimpleApplication::shootBox(const btVector3& destination) {

	if (m_dynamicsWorld) {
		float mass = 1.f;
		btTransform startTransform;
		startTransform.setIdentity();
		btVector3 camPos = getCameraPosition();
		startTransform.setOrigin(camPos);

		setShootBoxShape();

		btRigidBody* body = this->localCreateRigidBody(mass, startTransform,
				m_shootBoxShape);
		body->setLinearFactor(btVector3(1, 1, 1));
		//body->setRestitution(1);

		btVector3 linVel(destination[0] - camPos[0], destination[1] - camPos[1],
				destination[2] - camPos[2]);
		linVel.normalize();
		linVel *= m_ShootBoxInitialSpeed;

		body->getWorldTransform().setOrigin(camPos);
		body->getWorldTransform().setRotation(btQuaternion(0, 0, 0, 1));
		body->setLinearVelocity(linVel);
		body->setAngularVelocity(btVector3(0, 0, 0));
		body->setCcdMotionThreshold(0.5);
		body->setCcdSweptSphereRadius(0.9f);
//		printf("shootBox uid=%d\n", body->getBroadphaseHandle()->getUid());
//		printf("camPos=%f,%f,%f\n",camPos.getX(),camPos.getY(),camPos.getZ());
//		printf("destination=%f,%f,%f\n",destination.getX(),destination.getY(),destination.getZ());

	}
}

int gPickingConstraintId = 0;
btVector3 gOldPickingPos;
btVector3 gHitPos(-1, -1, -1);
float gOldPickingDist = 0.f;
btRigidBody* pickedBody = 0; //for deactivation state

btVector3 SimpleApplication::getRayTo(int x, int y) {

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

btRigidBody* SimpleApplication::localCreateRigidBody(float mass,
		const btTransform& startTransform, btCollisionShape* shape) {
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(
			startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape,
			localInertia);

	btRigidBody* body = new btRigidBody(cInfo);
	body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);
	body->setWorldTransform(startTransform);
#endif//
	m_dynamicsWorld->addRigidBody(body);

	return body;
}

//See http://www.lighthouse3d.com/opengl/glut/index.php?bmpfontortho
void SimpleApplication::setOrthographicProjection() {

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

void SimpleApplication::resetPerspectiveProjection() {

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	updateCamera();
}

void SimpleApplication::displayProfileString(int xOffset, int yStart,
		char* message) {
	glRasterPos3f(btScalar(xOffset), btScalar(yStart), btScalar(0));
	GLDebugDrawString(xOffset, yStart, message);
}

void SimpleApplication::showProfileInfo(int& xOffset, int& yStart, int yIncr) {
#ifndef BT_NO_PROFILE

	static double time_since_reset = 0.f;
	time_since_reset = CProfileManager::Get_Time_Since_Reset();

	{
		//recompute profiling data, and store profile strings

		char blockTime[128];

		double totalTime = 0;

		int frames_since_reset = CProfileManager::Get_Frame_Count_Since_Reset();

		m_profileIterator->First();

		double parent_time =
				m_profileIterator->Is_Root() ? time_since_reset :
						m_profileIterator->Get_Current_Parent_Total_Time();

		{
			sprintf(blockTime, "--- Profiling: %s (total running time: %.3f ms) ---",
					m_profileIterator->Get_Current_Parent_Name(), parent_time);
			displayProfileString(xOffset, yStart, blockTime);
			yStart += yIncr;
			sprintf(blockTime,
					"press (1,2...) to display child timings, or 0 for parent");
			displayProfileString(xOffset, yStart, blockTime);
			yStart += yIncr;

		}

		double accumulated_time = 0.f;

		for (int i = 0; !m_profileIterator->Is_Done(); m_profileIterator->Next()) {
			double current_total_time = m_profileIterator->Get_Current_Total_Time();
			accumulated_time += current_total_time;
			double fraction =
					parent_time > SIMD_EPSILON ? (current_total_time / parent_time)
							* 100 :
							0.f;

			sprintf(blockTime, "%d -- %s (%.2f %%) :: %.3f ms / frame (%d calls)",
					++i, m_profileIterator->Get_Current_Name(), fraction,
					(current_total_time / (double) frames_since_reset),
					m_profileIterator->Get_Current_Total_Calls());
			displayProfileString(xOffset, yStart, blockTime);
			yStart += yIncr;
			totalTime += current_total_time;
		}

		sprintf(
				blockTime,
				"%s (%.3f %%) :: %.3f ms",
				"Unaccounted",
				// (min(0, time_since_reset - totalTime) / time_since_reset) * 100);
				parent_time > SIMD_EPSILON ? ((parent_time - accumulated_time)
						/ parent_time) * 100 :
						0.f, parent_time - accumulated_time);

		displayProfileString(xOffset, yStart, blockTime);
		yStart += yIncr;

		sprintf(blockTime, "-------------------------------------------------");
		displayProfileString(xOffset, yStart, blockTime);
		yStart += yIncr;

	}
#endif//BT_NO_PROFILE
}

//
void SimpleApplication::renderscene(int pass) {
	btScalar m[16];
	btMatrix3x3 rot;
	rot.setIdentity();
	const int numObjects = m_dynamicsWorld->getNumCollisionObjects();
	btVector3 wireColor(1, 0, 0);
	for (int i = 0; i < numObjects; i++) {
		btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
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
		m_dynamicsWorld->getBroadphase()->getBroadphaseAabb(aabbMin, aabbMax);

		aabbMin -= btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
		aabbMax += btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
//		printf("aabbMin=(%f,%f,%f)\n",aabbMin.getX(),aabbMin.getY(),aabbMin.getZ());
//		printf("aabbMax=(%f,%f,%f)\n",aabbMax.getX(),aabbMax.getY(),aabbMax.getZ());
//		m_dynamicsWorld->getDebugDrawer()->drawAabb(aabbMin,aabbMax,btVector3(1,1,1));

		if (!(getDebugMode() & btIDebugDraw::DBG_DrawWireframe)) {
			switch (pass) {
			case 0:
				m_shapeDrawer->drawOpenGL(m, colObj->getCollisionShape(), wireColor,
						getDebugMode(), aabbMin, aabbMax);
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

//
void SimpleApplication::renderme() {
	myinit();

	updateCamera();

	if (m_dynamicsWorld) {
		if (m_enableshadows) {
			glClear(GL_STENCIL_BUFFER_BIT);
			glEnable(GL_CULL_FACE);
			renderscene(0);

			glDisable(GL_LIGHTING);
			glDepthMask(GL_FALSE);
			glDepthFunc(GL_LEQUAL);
			glEnable(GL_STENCIL_TEST);
			glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
			glStencilFunc(GL_ALWAYS, 1, 0xFFFFFFFFL);
			glFrontFace(GL_CCW);
			glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
			renderscene(1);
			glFrontFace(GL_CW);
			glStencilOp(GL_KEEP, GL_KEEP, GL_DECR);
			renderscene(1);
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
			renderscene(2);
			glEnable(GL_LIGHTING);
			glDepthFunc(GL_LESS);
			glDisable(GL_STENCIL_TEST);
			glDisable(GL_CULL_FACE);
		} else {
			glDisable(GL_CULL_FACE);
			renderscene(0);
		}

		int xOffset = 10;
		int yStart = 20;
		int yIncr = 20;

		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);

		if ((m_debugMode & btIDebugDraw::DBG_NoHelpText) == 0) {
			setOrthographicProjection();

			showProfileInfo(xOffset, yStart, yIncr);

			resetPerspectiveProjection();
		}

		glDisable(GL_LIGHTING);

	}

	updateCamera();

}

#include "BulletCollision/BroadphaseCollision/btAxisSweep3.h"

void SimpleApplication::clientResetScene() {
//	removePickingConstraint();

#ifdef SHOW_NUM_DEEP_PENETRATIONS
	gNumDeepPenetrationChecks = 0;
	gNumGjkChecks = 0;
#endif //SHOW_NUM_DEEP_PENETRATIONS
	gNumClampedCcdMotions = 0;
	int numObjects = 0;
	int i;

	if (m_dynamicsWorld) {
		int numConstraints = m_dynamicsWorld->getNumConstraints();
		for (i = 0; i < numConstraints; i++) {
			m_dynamicsWorld->getConstraint(0)->setEnabled(true);
		}
		numObjects = m_dynamicsWorld->getNumCollisionObjects();

		///create a copy of the array, not a reference!
		btCollisionObjectArray copyArray =
				m_dynamicsWorld->getCollisionObjectArray();

		for (i = 0; i < numObjects; i++) {
			btCollisionObject* colObj = copyArray[i];
			btRigidBody* body = btRigidBody::upcast(colObj);
			if (body) {
				if (body->getMotionState()) {
					btDefaultMotionState* myMotionState =
							(btDefaultMotionState*) body->getMotionState();
					myMotionState->m_graphicsWorldTrans =
							myMotionState->m_startWorldTrans;
					body->setCenterOfMassTransform(myMotionState->m_graphicsWorldTrans);
					colObj->setInterpolationWorldTransform(
							myMotionState->m_startWorldTrans);
					colObj->forceActivationState(ACTIVE_TAG);
					colObj->activate();
					colObj->setDeactivationTime(0);
					//colObj->setActivationState(WANTS_DEACTIVATION);
				}
				//removed cached contact points (this is not necessary if all objects have been removed from the dynamics world)
				if (m_dynamicsWorld->getBroadphase()->getOverlappingPairCache())
					m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(
							colObj->getBroadphaseHandle(),
							getDynamicsWorld()->getDispatcher());

				btRigidBody* body = btRigidBody::upcast(colObj);
				if (body && !body->isStaticObject()) {
					btRigidBody::upcast(colObj)->setLinearVelocity(btVector3(0, 0, 0));
					btRigidBody::upcast(colObj)->setAngularVelocity(btVector3(0, 0, 0));
				}
			}

		}

		///reset some internal cached data in the broadphase
		m_dynamicsWorld->getBroadphase()->resetPool(
				getDynamicsWorld()->getDispatcher());
		m_dynamicsWorld->getConstraintSolver()->reset();

	}

}

void SimpleApplication::updateModifierKeys() {
	m_modifierKeys = 0;
	if (glutGetModifiers() & GLUT_ACTIVE_ALT)
		m_modifierKeys |= BT_ACTIVE_ALT;

	if (glutGetModifiers() & GLUT_ACTIVE_CTRL)
		m_modifierKeys |= BT_ACTIVE_CTRL;

	if (glutGetModifiers() & GLUT_ACTIVE_SHIFT)
		m_modifierKeys |= BT_ACTIVE_SHIFT;
}

void SimpleApplication::specialKeyboard(int key, int x, int y) {
	(void) x;
	(void) y;

	switch (key) {
	case GLUT_KEY_F1: {

		break;
	}

	case GLUT_KEY_F2: {

		break;
	}

	case GLUT_KEY_END: {
		int numObj = getDynamicsWorld()->getNumCollisionObjects();
		if (numObj) {
			btCollisionObject* obj =
					getDynamicsWorld()->getCollisionObjectArray()[numObj - 1];

			getDynamicsWorld()->removeCollisionObject(obj);
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState()) {
				delete body->getMotionState();
			}
			delete obj;

		}
		break;
	}
	default:
		//        std::cout << "unused (special) key : " << key << std::endl;
		break;
	}

	glutPostRedisplay();

}

void SimpleApplication::swapBuffers() {
	glutSwapBuffers();
}
