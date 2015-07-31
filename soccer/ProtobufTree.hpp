#pragma once

#include <vector>
#include <memory>
#include <QTreeWidget>
#include <google/protobuf/message.h>

class QMainWindow;
class QTimer;

namespace Packet
{
	class LogFrame;
}

class ProtobufTree: public QTreeWidget
{
	Q_OBJECT;
	
	public:
		ProtobufTree(QWidget *parent = nullptr);
		
		// Column numbers
		enum
		{
			Column_Field = 0,
			Column_Value = 1,
			Column_Tag = 2
		};
		
		// Updates the tree with the given message.
		// Returns true if any items were added.
		//
		// Items will only be removed from the tree when the number of elements
		// in a repeated field is reduced.  Fields are never removed.
		bool message(const google::protobuf::Message &msg);
		
		void expandMessages(QTreeWidgetItem *item = nullptr);
		
		// Expands an item recursively
		void expandSubtree(QTreeWidgetItem *item);
		
		// Collapses an item recursively
		void collapseSubtree(QTreeWidgetItem *item);
		
		// This is only used for creating charts
		void history(const std::vector<std::shared_ptr<Packet::LogFrame> > *value)
		{
			_history = value;
		}
		
		QMainWindow *mainWindow;
		QTimer *updateTimer;
		
	protected:
		// Recursively updates the tree.
		// This should only be called for items where <parent> is the item for a message (not a repeated field).
		bool addTreeData(QTreeWidgetItem *parent, const google::protobuf::Message &msg);
		
		void addBytes(QTreeWidgetItem *parent, const std::string &bytes);
		
		virtual void contextMenuEvent(QContextMenuEvent *e);
		
		bool _first;
		const std::vector<std::shared_ptr<Packet::LogFrame> > *_history;
};
