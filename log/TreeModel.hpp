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
			TreeModel();
			
		protected Q_SLOTS:
			void frame(Packet::LogFrame* frame);
			
		private:
	};
}
