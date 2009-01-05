#ifndef VISION_HPP_
#define VISION_HPP_

#include <QThread>
#include <QMutex>
#include <Network/PacketReceiver.hpp>


#include "Physics/Env.hpp"

class SimVision : public QThread
{
	public:
		SimVision(Env* env);
		~SimVision();

                uint64_t timestamp();
                void radioHandler(const Packet::RadioTx* packet);

	protected:
		void run();

	private:

	    Env* _env;

	    unsigned int _id, _fps;

            QMutex _simVisionMutex;

            /** Reciever for radio commands. Temporary until something better comes along**/
	    Network::PacketReceiver* _receiver;

	    //const Packet::RadioTx* _txPacket;
};

#endif /* VISION_HPP_ */
