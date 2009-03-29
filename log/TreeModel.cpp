#include "TreeModel.hpp"

#include <LogFrame.hpp>

using namespace Log;

TreeModel::TreeModel()
{
	//_timestamp = new QStandardItem();
	//this->appendRow(_timestamp);
#if 0
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
#endif
}

#include <tree_Point2d.hpp>
#include <tree_Vision.hpp>
#include <tree_RadioTx.hpp>
#include <tree_RadioRx.hpp>
#include <tree_LogFrame.hpp>

void TreeModel::frame(Packet::LogFrame* frame)
{
	//TODO make root node, not timestamp
	Packet::LogFrame& f = *frame;
	Log::handleExt(this->invisibleRootItem(), f);
	
#if 0
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
#endif
}

void TreeModel::robotItem(QStandardItem* item, const Packet::Vision::Robot& r)
{
#if 0
	for (int i=item->rowCount() ; i<2 ; ++i)
	{
		item->appendRow(new QStandardItem());
	}
	
	item->setText(QString("Robot: %1").arg(r.shell));
	
	QString pos = QString("Pos: %1, %2").arg(r.pos.x).arg(r.pos.y);
	item->child(0)->setText(pos);
	item->child(1)->setText(QString("Angle: %1").arg(r.angle));
#endif
}

void TreeModel::robotItem(QStandardItem* item, const Packet::LogFrame::Robot& r)
{
#if 0
	for (int i=item->rowCount() ; i<6 ; ++i)
	{
		item->appendRow(new QStandardItem());
	}
	
	item->setText(QString("Robot: %1 (%2)").arg(r.shell).arg(r.valid));
	
	int c = 0;
	//item->child(c++)->setText(QString("Pos: %1, %2").arg(r.pos.x).arg(r.pos.y));
	Log::handleExt(item->child(c++), r.pos);
	/*item->child(c++)->setText(QString("Vel: %1, %2").arg(r.vel.x).arg(r.vel.y));
	item->child(c++)->setText(QString("Angle: %1").arg(r.angle));
	item->child(c++)->setText(QString("AngVel: %1").arg(r.angleVel));
	*/
	///radio tx data
	//handleExt(item->child(c++), r.radioTx);
	/*
	QStandardItem* rTx = item->child(c++);
	rTx->setText(QString("Radio Tx (%1)").arg(r.radioTx.valid));
	
	for (int i=rTx->rowCount() ; i<7 ; ++i)
	{
		rTx->appendRow(new QStandardItem());
	}
	
	int c1 = 0;
	rTx->child(c1++)->setText(QString("Board: %1").arg(r.radioTx.board_id));
	rTx->child(c1++)->setText(QString("Roller: %1").arg(r.radioTx.roller));
	rTx->child(c1++)->setText(QString("Kick: %1").arg(r.radioTx.kick));
	
	for (int i=0 ; i<4 ; ++i)
	{
		rTx->child(c1++)->setText(QString("M %1: %2").arg(i).arg(r.radioTx.motors[i]));
	}
	*/
	///radio rx data
	QStandardItem* rRx = item->child(c++);
	rRx->setText(QString("Radio Rx (%1)").arg(r.radioRx.valid));
	
	for (int i=rRx->rowCount() ; i<16 ; ++i)
	{
		rRx->appendRow(new QStandardItem());
	}
	
	int c1 = 0;
	rRx->child(c1++)->setText(QString("Updated: %1").arg(r.radioRx.updated));
	rRx->child(c1++)->setText(QString("Loss: %1").arg(r.radioRx.packetLoss));
	rRx->child(c1++)->setText(QString("Rssi: %1").arg(r.radioRx.rssi));
	rRx->child(c1++)->setText(QString("Tuning: %1").arg(r.radioRx.tuning));
	rRx->child(c1++)->setText(QString("Battery: %1").arg(r.radioRx.battery));
	rRx->child(c1++)->setText(QString("Capacitor: %1").arg(r.radioRx.capacitor));
	
	for (int i=0 ; i<5 ; ++i)
	{
		rRx->child(c1++)->setText(QString("M %1: %2").arg(i).arg(r.radioRx.motorFault[i]));
		rRx->child(c1++)->setText(QString("Enc %1: %2").arg(i).arg(r.radioRx.encoders[i]));
	}
#endif
}

void TreeModel::ballItem(QStandardItem* item, const Packet::Vision::Ball& b)
{
#if 0
	QString pos = QString("Pos: %1, %2").arg(
		QString::number(b.pos.x)).arg(QString::number(b.pos.y));
	
	item->setText(pos);
#endif
}
