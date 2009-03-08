#include "TreeModel.hpp"

#include <LogFrame.hpp>
#include <boost/foreach.hpp>

using namespace Log;

TreeModel::TreeModel()
{
	_timestamp = new QStandardItem();
	this->appendRow(_timestamp);
	
	_vision = new QStandardItem("raw vision");
	_timestamp->appendRow(_vision);
	
	_self = new QStandardItem("self");
	_timestamp->appendRow(_self);
	_opp = new QStandardItem("opp");
	_timestamp->appendRow(_opp);
	
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		_self->appendRow(new QStandardItem(QString("Player ") + QString::number(i)));
		_opp->appendRow(new QStandardItem(QString("Player ") + QString::number(i)));
	}
	
	_vision->appendRow(new QStandardItem("Self"));
	_vision->appendRow(new QStandardItem("Opp"));
	_vision->appendRow(new QStandardItem("Balls"));
}

void TreeModel::frame(Packet::LogFrame* frame)
{
	_timestamp->setData(QString::number(frame->timestamp), Qt::DisplayRole);
	
	QStandardItem* rItem = _vision->child(0);
	rItem->removeRows(0, rItem->rowCount());
	//BOOST_FOREACH(const Packet::Vision::Robot& r, frame->allSelf)
	{
		/*
		QStandardItem* robot = new QStandardItem(QString("Shell: ") + 
				QString::number(r.shell));
		
		QString pos = "Pos: ";
		pos += QString::number(r.pos.x);
		pos += ", " + QString::number(r.pos.y);
		
		robot->appendRow(new QStandardItem(pos));
		rItem->appendRow(robot);
		*/
	}
	
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		QStandardItem* rItem = _self->child(i);
		rItem->removeRows(0, rItem->rowCount());
		
		QString pos = "Pos: ";
		pos += QString::number(frame->self[i].pos.x);
		pos += ", " + QString::number(frame->self[i].pos.y);
		
		rItem->appendRow(new QStandardItem(pos));
	}
}
