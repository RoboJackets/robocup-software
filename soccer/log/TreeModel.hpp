#pragma once

#include <QStandardItemModel>
#include <QStandardItem>

#include <LogFrame.hpp>

namespace Log
{
	class TreeModel : public QStandardItemModel
	{
		public:
			TreeModel(QObject *parent = 0);
			
			void frame(Packet::LogFrame* frame);
	};
}
