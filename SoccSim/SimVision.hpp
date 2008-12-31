#ifndef VISION_HPP_
#define VISION_HPP_

#include <QThread>

#include "Physics/Env.hpp"

class SimVision : public QThread
{
	public:
		SimVision(Env* env);
		~SimVision();

                uint64_t timestamp();

	protected:
		void run();

	private:

	    Env* _env;

	    unsigned int _id, _fps;
};

#endif /* VISION_HPP_ */
