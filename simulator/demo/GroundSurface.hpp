#pragma once

class btTriangleIndexVertexArray;
class SimEngine;
class btVector3;

class GroundSurface {
protected:
	// Terrain components
	class btTriangleIndexVertexArray* _indexVertexArrays;
	btVector3* _vertices;

	// links to the engine
	SimEngine *_simEngine;

public:

	GroundSurface(SimEngine *engine);

	~GroundSurface();

	void initPhysics();
};

