#include "TreeView.hpp"
#include "TreeModel.hpp"

#include <QHeaderView>

TreeView::TreeView(QWidget* parent) :
	QTreeView(parent)
{
	header()->setVisible(false);
}

