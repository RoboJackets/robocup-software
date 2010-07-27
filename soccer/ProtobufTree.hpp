#pragma once

#include <QTreeWidget>
#include <google/protobuf/message.h>

class ProtobufTree: public QTreeWidget
{
	Q_OBJECT;
	
	public:
		ProtobufTree(QWidget *parent = 0);
		
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
		
		void expandMessages(QTreeWidgetItem *item = 0);
		
		// Expands an item recursively
		void expandSubtree(QTreeWidgetItem *item);
		
		// Collapses an item recursively
		void collapseSubtree(QTreeWidgetItem *item);		
		
	protected:
		// Recursively updates the tree.
		// This should only be called for items where <parent> is the item for a message (not a repeated field).
		bool addTreeData(QTreeWidgetItem *parent, const google::protobuf::Message &msg);
		
		void addBytes(QTreeWidgetItem *parent, const std::string &bytes);
		
		virtual void contextMenuEvent(QContextMenuEvent *e);
};
