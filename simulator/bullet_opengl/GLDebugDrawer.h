#ifndef GL_DEBUG_DRAWER_H
#define GL_DEBUG_DRAWER_H

#include "LinearMath/btIDebugDraw.h"



class GLDebugDrawer : public btIDebugDraw
{
	int m_debugMode;

public:

	GLDebugDrawer();
	virtual ~GLDebugDrawer(); 

	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& fromColor, const btVector3& toColor) override;

	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color) override;

	virtual void	drawSphere (const btVector3& p, btScalar radius, const btVector3& color) override;

	virtual void	drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha) override;
	
	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color) override;

	virtual void	reportErrorWarning(const char* warningString) override;

	virtual void	draw3dText(const btVector3& location,const char* textString) override;

	virtual void	setDebugMode(int debugMode) override;

	virtual int		getDebugMode() const override { return m_debugMode;}

};

#endif//GL_DEBUG_DRAWER_H
