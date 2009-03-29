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
			
		protected Q_SLOTS:
			void frame(Packet::LogFrame* frame);
			
		private:
	};
}

#endif /* TREEMODEL_HPP_ */
