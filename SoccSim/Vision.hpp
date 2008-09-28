#ifndef VISION_HPP_
#define VISION_HPP_

#include <QThread>

#include "Physics/Env.hpp"

class Vision : public QThread
{
	public:
		Vision(Env* env);
		~Vision();
		
	protected:
		void run();
		
	private:
		
		Env* _env;
};

#endif /* VISION_HPP_ */
