#ifndef VISION_HPP 
#define VISION_HPP

#include <QThread>
#include <QUdpSocket>
#include <QMutex>

#include "Physics/Env.hpp"

/** Handles the boradcast of vision information for the client */
class Vision : public QThread
{
	public:
		Vision(Physics::Env& env);
		~Vision();
		
		float err(const float stdDev);
		
		void disableError() { _error = false; }
		void enableError() { _error = true; }
		
	protected:
		void run();
		
		static float clip(const float in, const float min, const float max);
		
		Physics::Env& _env;
		
	private:
		bool _running;
		
		const float _ballStdDev;
		const float _blueStdDev;
		const float _yellowStdDev;
		
		bool _error;
		
		static const int MaxMMError = 3;
};

#endif
