#ifndef TREEMODEL_HPP_
#define TREEMODEL_HPP_

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
			
		private:
			void rawVisionItem(QStandardItem* item, const Packet::Vision& vis);
			void robotItem(QStandardItem* item, const Packet::Vision::Robot& r);
			void ballItem(QStandardItem* item, const Packet::Vision::Ball& b);
			
			void robotItem(QStandardItem* item, const Packet::LogFrame::Robot& r);
			
		protected Q_SLOTS:
			void frame(Packet::LogFrame* frame);
			
		private:
			QStandardItem* _timestamp;
			QStandardItem* _teamName;
			
			//TODO ref stuff
			//QStandardItem* _controlMode;
			//QStandardItem* _runMode;
			
			QStandardItem* _rawVision;
			QStandardItem* _self;
			QStandardItem* _opp;
	};
}

#endif /* TREEMODEL_HPP_ */
