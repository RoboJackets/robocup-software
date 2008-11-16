#include "TreeModel.hpp"

#include <LogFrame.hpp>

using namespace Log;

TreeModel::TreeModel()
{
	_timestamp = new QStandardItem();
	this->appendRow(_timestamp);
	
	_vision = new QStandardItem("vision");
	_timestamp->appendRow(_vision);
}

void TreeModel::frame(Packet::LogFrame* frame)
{
	_timestamp->setData(QString::number(frame->vision.timestamp), Qt::DisplayRole);
	
	_vision->removeRows(0, _vision->rowCount());
	int i=0;
	Q_FOREACH(const Packet::LocVision::Robot& r, frame->vision.self)
	{
		QStandardItem* robot = new QStandardItem(QString::number(r.pos.x));
		_vision->setChild(i++, robot);
	}
}
