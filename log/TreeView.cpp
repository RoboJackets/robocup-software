#include "TreeView.hpp"
#include "TreeModel.hpp"

#include <QHeaderView>

using namespace Log;

TreeView::TreeView(QWidget* parent) :
	QTreeView(parent)
{
	header()->setVisible(false);
}

