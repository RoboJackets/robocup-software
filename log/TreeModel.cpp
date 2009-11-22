#include "TreeModel.hpp"

#include <LogFrame.hpp>

using namespace Log;

TreeModel::TreeModel()
{
	this->invisibleRootItem()->setEditable(false);
	this->invisibleRootItem()->setColumnCount(2);
}

#include <tree_Point.hpp>
#include <tree_Segment.hpp>
#include <tree_Polygon.hpp>
#include <tree_Vision.hpp>
#include <tree_RadioTx.hpp>
#include <tree_RadioRx.hpp>
#include <tree_GameState.hpp>
#include <tree_MotionCmd.hpp>
#include <tree_LogFrame.hpp>

void TreeModel::frame(Packet::LogFrame* frame)
{
	Log::handleExt(this->invisibleRootItem(), this->invisibleRootItem(), *frame);
}

