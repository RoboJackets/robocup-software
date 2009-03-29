#ifndef VISION_HPP_
#define VISION_HPP_

#include "Physics/Env.hpp"
#include <Network/PacketReceiver.hpp>
#include <QThread>

/** vision interface class, emulates the vision system for the simulator */
class VisionGen: public QThread
{
	public:
		VisionGen(Env* env, unsigned int id);
		~VisionGen();
        
	protected:
		void run();

	private:
		Env* _env;
		
		bool _running;

		unsigned int _id, _fps;
};
#endif /* VISION_HPP_ */
