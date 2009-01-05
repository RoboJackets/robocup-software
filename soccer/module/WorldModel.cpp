#include "WorldModel.hpp"

#include <QObject>

WorldModel::WorldModel() :
	Module("World Model")
{

}

void WorldModel::run()
{
	Q_FOREACH(const Packet::Vision& vision, _state->rawVision)
	{
		//printf("Number of bots %d\n", vision.blue.size());

		if (!vision.sync)
		{
			//index is the id
			Q_FOREACH (const Packet::Vision::Robot& r, vision.blue)
			{
                            _state->self[r.shell].shell = r.shell;
                            _state->self[r.shell].pos = r.pos;
        ;                   _state->self[r.shell].angle = r.angle;
                            _state->self[r.shell].valid = true;
			}
		}
	}
}
