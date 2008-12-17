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
		if (!vision.sync)
		{
			//index is the id
			Q_FOREACH (const Packet::Vision::Robot& r, vision.blue)
			{
				Packet::LogFrame::Robot& rb = _state->self[r.shell % 5];
				rb.shell = r.shell;
				rb.valid = true;

				rb.pos = r.pos;
				rb.angle = r.angle;
			}
		}
	}
}
