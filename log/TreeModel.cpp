#include "TreeModel.hpp"

#include <LogFrame.hpp>

using namespace Log;

TreeModel::TreeModel()
{
	/*
	QStandardItem* r = this->invisibleRootItem();
	
	QStandardItem* i11 = new QStandardItem("i11");
	QStandardItem* i12 = new QStandardItem("i12");
	
	QStandardItem* i21 = new QStandardItem("i21");
	QStandardItem* i22 = new QStandardItem("i22");
	
	QStandardItem* i31 = new QStandardItem("i31");
	QStandardItem* i32 = new QStandardItem("i32");
	
	r->setChild(0,0, i11);
	r->setChild(0,1, i12);
	
	r->setChild(1,0, i21);
	r->setChild(1,1, i22);
	
	i11->setChild(0,0, i31);
	i11->setChild(0,1, i32);
	*/
	
	this->invisibleRootItem()->setColumnCount(2);
}

#include <tree_Point2d.hpp>
#include <tree_Vision.hpp>
#include <tree_RadioTx.hpp>
#include <tree_RadioRx.hpp>
#include <tree_LogFrame.hpp>

void TreeModel::frame(Packet::LogFrame* frame)
{
	Log::handleExt(this->invisibleRootItem(), this->invisibleRootItem(), *frame);
}

