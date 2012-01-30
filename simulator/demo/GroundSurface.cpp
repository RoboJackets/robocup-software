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

#include <boost/tuple/tuple.hpp>

#include "GlutCamera.hpp"
#include "GroundSurface.hpp"

#include <btBulletDynamicsCommon.h>

#include "GLDebugDrawer.h"
#include <stdio.h> //printf debugging
#include "GL_ShapeDrawer.h"

const int maxProxies = 32766;
const int maxOverlap = 65535;

///btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
///notice that for higher-quality slow-moving vehicles, another approach might be better
///implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts

#define CUBE_HALF_EXTENTS 1

using namespace std;

void GroundSurface::initPhysics()
{
	btCollisionShape* groundShape = new btBoxShape(btVector3(50, 3, 50));
	_simEngine->addCollisionShape(groundShape);
	btTransform tr;
	tr.setIdentity();

	// use triangle mesh for ground
	int i;

	const float TRIANGLE_SIZE = 20.f;

	//create a triangle-mesh ground
	int vertStride = sizeof(btVector3);
	int indexStride = 3 * sizeof(int);

	const int NUM_VERTS_X = 20;
	const int NUM_VERTS_Y = 20;
	const int totalVerts = NUM_VERTS_X * NUM_VERTS_Y;

	const int totalTriangles = 2 * (NUM_VERTS_X - 1) * (NUM_VERTS_Y - 1);

	_vertices = new btVector3[totalVerts];
	int* gIndices = new int[totalTriangles * 3];

	for (i = 0; i < NUM_VERTS_X; i++) {
		for (int j = 0; j < NUM_VERTS_Y; j++) {
			float height = 0.f;
			_vertices[i + j * NUM_VERTS_X].setValue(
					(i - NUM_VERTS_X * 0.5f) * TRIANGLE_SIZE, height,
					(j - NUM_VERTS_Y * 0.5f) * TRIANGLE_SIZE);
		}
	}

	int index = 0;
	for (i = 0; i < NUM_VERTS_X - 1; i++) {
		for (int j = 0; j < NUM_VERTS_Y - 1; j++) {
			gIndices[index++] = j * NUM_VERTS_X + i;
			gIndices[index++] = j * NUM_VERTS_X + i + 1;
			gIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;

			gIndices[index++] = j * NUM_VERTS_X + i;
			gIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;
			gIndices[index++] = (j + 1) * NUM_VERTS_X + i;
		}
	}

	_indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles, gIndices,
			indexStride, totalVerts, (btScalar*) &_vertices[0].x(), vertStride);

	bool useQuantizedAabbCompression = true;
	groundShape = new btBvhTriangleMeshShape(_indexVertexArrays,
			useQuantizedAabbCompression);

	tr.setOrigin(btVector3(0, -4.5f, 0));

	_simEngine->addCollisionShape(groundShape);

	//create ground object
	_simEngine->localCreateRigidBody(0, tr, groundShape);
}

GroundSurface::GroundSurface(SimEngine *engine)
: _indexVertexArrays(0), _vertices(0), _simEngine(engine)
{
}

GroundSurface::~GroundSurface() {
	delete _indexVertexArrays;
	delete _vertices;
}

