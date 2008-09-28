#ifndef _ENV_HPP
#define _ENV_HPP

#include <NxPhysics.h>

#include <QVector>
#include <QTimer>

class Entity;

class Env : public QObject
{
	Q_OBJECT;
	
	public:
		Env();
		~Env();

		/** return the debug renderable for the environment */
		const NxDebugRenderable& dbgRenderable() const;
		
		void start();
		
	private Q_SLOTS:
		void step();
		
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
		
		QTimer _step;
};

#endif /* _ENV_HPP */
