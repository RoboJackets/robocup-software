#include "TreeModel.hpp"

#include <LogFrame.hpp>

using namespace Log;

TreeModel::TreeModel()
{
	_timestamp = new QStandardItem();
	this->appendRow(_timestamp);
	
	_teamName = new QStandardItem();
	this->appendRow(_teamName);
	
	_rawVision = new QStandardItem("raw vision");
	this->appendRow(_rawVision);
	
	_self = new QStandardItem("self");
	this->appendRow(_self);
	
	_opp = new QStandardItem("opp");
	this->appendRow(_opp);
	
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		_self->appendRow(new QStandardItem());
		_opp->appendRow(new QStandardItem());
	}
}

void TreeModel::frame(Packet::LogFrame* frame)
{
	_teamName->setText(QString("Team: %1").arg(teamName(frame->team)));
	
	QString ts = QString::number((quint64)frame->timestamp);
	_timestamp->setData(QString("%1.%2").arg(ts.left(ts.size()-6)).arg(ts.right(6)),
		Qt::DisplayRole);
	
	int r1 = 0;	
	Q_FOREACH(const Packet::Vision& vis, frame->rawVision)
	{
		QStandardItem* packet = _rawVision->child(r1++);
		
		if (!packet)
		{
			packet = new QStandardItem("Packet");
			
			//timestamp, sync, camera
			packet->appendRow(new QStandardItem());
			packet->appendRow(new QStandardItem());
			packet->appendRow(new QStandardItem());
			
			packet->appendRow(new QStandardItem("Self"));
			packet->appendRow(new QStandardItem("Opp"));
			packet->appendRow(new QStandardItem("Ball"));
			
			_rawVision->appendRow(packet);
		}
		
		//timestamp
		QString ts = QString::number((quint64)vis.timestamp);
		packet->child(0)->setText(QString("%1.%2").arg(
			ts.left(ts.size()-6)).arg(ts.right(6)));
		
		//sync
		packet->child(1)->setText(QString("sync: %1").arg(vis.sync));
		
		//camera
		packet->child(2)->setText(QString("camera: %1").arg(vis.camera));
		
		if (vis.sync)
		{
			packet->removeRows(3, packet->rowCount() - 3);
			continue;
		}
		
		if (packet->rowCount() == 3)
		{
			packet->appendRow(new QStandardItem("Self"));
			packet->appendRow(new QStandardItem("Opp"));
			packet->appendRow(new QStandardItem("Ball"));
		}
		
		QStandardItem* self = packet->child(3);
		QStandardItem* opp = packet->child(4);
		QStandardItem* ball = packet->child(5);
		
		int r2 = 0; 
		Q_FOREACH(const Packet::Vision::Robot& r, vis.blue)
		{
			QStandardItem* item = self->child(r2++);
			if (!item)
			{
				item = new QStandardItem();
				self->appendRow(item);
			}
			
			robotItem(item, r);
		}
		
		self->removeRows(r2, self->rowCount() - r2);
		
		r2 = 0; 
		Q_FOREACH(const Packet::Vision::Robot& r, vis.yellow)
		{
			QStandardItem* item = opp->child(r2++);
			if (!item)
			{
				item = new QStandardItem();
				opp->appendRow(item);
			}
			
			robotItem(item, r);
		}
		
		opp->removeRows(r2, opp->rowCount() - r2);
		
		r2 = 0; 
		Q_FOREACH(const Packet::Vision::Ball& b, vis.balls)
		{
			
			QStandardItem* item = ball->child(r2++);
			if (!item)
			{
				item = new QStandardItem();
				ball->appendRow(item);
			}
			
			ballItem(item, b);
		}
		
		ball->removeRows(r2, ball->rowCount() - r2);
	}
	
	_rawVision->removeRows(r1, _rawVision->rowCount() - r1);
	
	for (unsigned int i=0 ; i<5 ; ++i)
	{
		robotItem(_self->child(i), frame->self[i]);
		robotItem(_opp->child(i), frame->opp[i]);
	}
}

void TreeModel::robotItem(QStandardItem* item, const Packet::Vision::Robot& r)
{
	for (int i=item->rowCount() ; i<2 ; ++i)
	{
		item->appendRow(new QStandardItem());
	}
	
	item->setText(QString("Robot: %1").arg(r.shell));
	
	QString pos = QString("Pos: %1, %2").arg(r.pos.x).arg(r.pos.y);
	item->child(0)->setText(pos);
	item->child(1)->setText(QString("Angle: %1").arg(r.angle));
}

void TreeModel::robotItem(QStandardItem* item, const Packet::LogFrame::Robot& r)
{
	for (int i=item->rowCount() ; i<5 ; ++i)
	{
		item->appendRow(new QStandardItem());
	}
	
	item->setText(QString("Robot: %1 (%2)").arg(r.shell).arg(r.valid));
	
	int c = 0;
	item->child(c++)->setText(QString("Pos: %1, %2").arg(r.pos.x).arg(r.pos.y));
	item->child(c++)->setText(QString("Vel: %1, %2").arg(r.vel.x).arg(r.vel.y));
	item->child(c++)->setText(QString("Angle: %1").arg(r.angle));
	item->child(c++)->setText(QString("AngVel: %1").arg(r.angleVel));
	
	QStandardItem* rtx = item->child(c++);
	rtx->setText(QString("Radio Tx (%1)").arg(r.radioTx.valid));
	
	for (int i=rtx->rowCount() ; i<7 ; ++i)
	{
		rtx->appendRow(new QStandardItem());
	}
	
	c = 0;
	rtx->child(c++)->setText(QString("Board: %1").arg(r.radioTx.board_id));
	rtx->child(c++)->setText(QString("Roller: %1").arg(r.radioTx.roller));
	rtx->child(c++)->setText(QString("Kick: %1").arg(r.radioTx.kick));
	
	for (int i=0 ; i<4 ; ++i)
	{
		rtx->child(c++)->setText(QString("M%1: %2").arg(i).arg(r.radioTx.motors[i]));
	}
}

void TreeModel::ballItem(QStandardItem* item, const Packet::Vision::Ball& b)
{
	QString pos = QString("Pos: %1, %2").arg(
		QString::number(b.pos.x)).arg(QString::number(b.pos.y));
	
	item->setText(pos);
}
