#ifndef _ENV_HPP
#define _ENV_HPP

#include <NxPhysics.h>

#include <QThread>
#include <QVector>

class Entity;

class Env : public QThread
{
	public:
		Env();
		~Env();

		/** return the debug renderable for the environment */
		const NxDebugRenderable& dbgRenderable() const;
		
		void redraw() const;
		
		void step();
		
	protected:
	    virtual void run();
		
	public:
	    /** the global physics sdk for every environment */
		static NxPhysicsSDK* _physicsSDK;
		
	private:
		/** number of open environments */
		static unsigned int _refCount;
		
		/** local scene for this environment */
		NxScene* _scene;
		
		/** all of the created entities for the environment */
		QVector<Entity*> _entities;
};

#endif /* _ENV_HPP */
