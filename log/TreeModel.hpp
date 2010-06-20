#pragma once

#include <QStandardItemModel>
#include <QStandardItem>

#include <LogFrame.hpp>

namespace Log
{
	class TreeModel : public QStandardItemModel
	{
		Q_OBJECT;
		
		public:
			TreeModel(QObject *parent = 0);
			
			void frame(Packet::LogFrame* frame);
			
		private:
	};
}
