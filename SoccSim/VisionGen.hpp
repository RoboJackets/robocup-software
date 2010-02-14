#pragma once

#include "Physics/Env.hpp"

#include <QThread>

/** vision interface class, emulates the vision system for the simulator */
class VisionGen: public QThread
{
	public:
		VisionGen(Env* env, unsigned int id, bool useNoisy = false);
		~VisionGen();

	protected:
		void run();

		// adds in place noise by changing values - also max speed on ball
		void addNoise(Packet::Vision& vision);

	private:
		bool _useNoisy;

		Env* _env;
		
		bool _running;

		unsigned int _id, _fps;

		// old vision packet - used for determining speed
		Packet::Vision _oldPacket;
};
