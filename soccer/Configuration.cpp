#include "Configuration.hpp"

#include <QTreeWidget>
#include <QTreeWidgetItem>

#include <stdio.h>
#include <assert.h>
#include <boost/foreach.hpp>

// Role for tree column zero for storing ConfigItem pointers.
static const int ConfigItemRole = Qt::UserRole;

Q_DECLARE_METATYPE(ConfigItem *)

ConfigItem::ConfigItem(Configuration *config, const QString& name)
{
	_config = config;
	_treeItem = 0;
	_path = name.split('/');
}

void ConfigItem::valueChanged(const QString& str)
{
	if (_treeItem)
	{
		_treeItem->setText(1, str);
	}
}

void ConfigItem::addToTree()
{
	_config->addItem(this);
}

////////

ConfigInt::ConfigInt(Configuration *config, QString name, int value):
	ConfigItem(config, name)
{
	_value = value;
	addToTree();
}

QString ConfigInt::toString()
{
	return QString::number(_value);
}

void ConfigInt::setValue(const QString& str)
{
	_value = str.toInt();
}

////////

ConfigDouble::ConfigDouble(Configuration *config, QString name, double value):
	ConfigItem(config, name)
{
	_value = value;
	addToTree();
}

QString ConfigDouble::toString()
{
	return QString::number(_value);
}

void ConfigDouble::setValue(const QString& str)
{
	_value = str.toDouble();
}

////////

ConfigFloatVector::ConfigFloatVector(Configuration *config, QString name):
	ConfigItem(config, name)
{
	addToTree();
}

void ConfigFloatVector::setValue(const QString& str)
{
	// This happens either when the user changes the value of the vector's item
	// (number of elements) or when the item is first created by addToTree().
	// In either case, it creates child items as needed.
	resize(str.toInt());
}

QString ConfigFloatVector::toString()
{
	return QString::number(_values.size());
}

void ConfigFloatVector::resize(unsigned int n)
{
	if (_treeItem)
	{
		_treeItem->setText(1, QString::number(n));
		
		// Delete extra items
		for (unsigned int i = n; i < _items.size(); ++i)
		{
			delete _items[i];
		}
	}
	
	unsigned int oldSize = _items.size();
	_values.resize(n);
	
	if (_treeItem)
	{
		_items.resize(n);
		
		// Create new items
		for (unsigned int i = oldSize; i < n; ++i)
		{
			_items[i] = new QTreeWidgetItem(_treeItem);
			_items[i]->setText(0, QString::number(i));
			_items[i]->setText(1, QString::number(_values[i]));
		}
	}
}

void ConfigFloatVector::set(unsigned int i, float value)
{
	_values[i] = value;
	
	if (_treeItem)
	{
		_items[i]->setText(1, QString::number(value));
	}
}

////////

Configuration::Configuration()
{
	_tree = 0;
	
	// Create the XML root element
	_doc.appendChild(_doc.createElement("config"));
}

void Configuration::addItem(ConfigItem* item)
{
	_allItems.push_back(item);
	
	if (_tree)
	{
		addToTree(item);
	}
}

void Configuration::addToTree(ConfigItem *item)
{
	// Find the parent of this item by following the tree through all but the last item in the path
	QTreeWidgetItem *parent = _tree->invisibleRootItem();
	const QStringList &path = item->path();
	QStringList::const_iterator last = path.end();
	--last;
	for (QStringList::const_iterator i = path.begin(); i != last; ++i)
	{
		QTreeWidgetItem *next = 0;
		for (int j = 0; j < parent->childCount(); ++j)
		{
			QTreeWidgetItem *child = parent->child(j);
			if (child->text(0) == *i)
			{
				next = child;
				break;
			}
		}
		
		if (!next)
		{
			// Create this item
			next = new QTreeWidgetItem(parent);
			next->setText(0, *i);
		}
		
		parent = next;
	}
	
	// Create a tree item
	item->_treeItem = new QTreeWidgetItem(parent);
	item->_treeItem->setFlags(item->_treeItem->flags() | Qt::ItemIsEditable);
	item->_treeItem->setData(0, ConfigItemRole, QVariant::fromValue(item));
	item->_treeItem->setText(0, path.back());
	item->_treeItem->setText(1, item->toString());
}

void Configuration::tree(QTreeWidget* tree)
{
	assert(!_tree);
	
	_tree = tree;
	connect(_tree, SIGNAL(itemChanged(QTreeWidgetItem *, int)), this, SLOT(itemChanged(QTreeWidgetItem *, int)));
	
	// Add items that were created before we got a tree
	BOOST_FOREACH(ConfigItem *item, _allItems)
	{
		addToTree(item);
	}
}

ConfigItem* Configuration::configItem(QTreeWidgetItem* ti)
{
	return ti->data(0, ConfigItemRole).value<ConfigItem *>();
}

void Configuration::itemChanged(QTreeWidgetItem* item, int column)
{
	if (column == 1)
	{
		ConfigItem *ci = configItem(item);
		if (ci)
		{
			ci->setValue(item->text(1));
		}
	}
}

bool Configuration::load(const QString &filename, QString &error)
{
	QFile file(filename);
	if (!file.open(QFile::ReadOnly))
	{
		error = file.errorString();
		return false;
	}
	
	QDomDocument newDoc;
	QString domError;
	int errorLine = 0, errorColumn = 0;
	if (!newDoc.setContent(&file, &domError, &errorLine, &errorColumn))
	{
		error = QString("%1:%2: %3").arg(
			QString::number(errorLine),
			QString::number(errorColumn),
			domError);
		return false;
	}
	
	QDomElement root = newDoc.firstChildElement("config");
	if (root.isNull())
	{
		error = "XML does not contain root <config> element";
		return false;
	}
	
	_doc = newDoc;
	BOOST_FOREACH(ConfigItem *item, _allItems)
	{
		QDomElement el = root;
		BOOST_FOREACH(QString str, item->path())
		{
			bool isInt = false;
			int value = str.toInt(&isInt);
			if (isInt)
			{
				str = QString("item%1").arg(value);
			} else {
				// Sanitize the string
				str.replace(' ', '_');
			}
			
			// Find the element for the next part of the path
			el = el.firstChildElement(str);
			if (el.isNull())
			{
				break;
			}
		}
		
		if (!el.isNull())
		{
			QString str = el.attribute("value");
			if (!str.isNull())
			{
				item->setValue(str);
			}
		}
	}
	
	return true;
}

bool Configuration::save(const QString &filename, QString &error)
{
	QFile file(filename);
	if (!file.open(QFile::WriteOnly))
	{
		error = file.errorString();
		return false;
	}
	
	QDomElement root = _doc.firstChildElement("config");
	
	// Update the DOM
	BOOST_FOREACH(ConfigItem *item, _allItems)
	{
		QDomElement el = root;
		BOOST_FOREACH(QString str, item->path())
		{
			bool isInt = false;
			int value = str.toInt(&isInt);
			if (isInt)
			{
				str = QString("item%1").arg(value);
			} else {
				// Sanitize the string
				str.replace(' ', '_');
			}
			
			// Find the element for the next part of the path
			QDomElement child = el.firstChildElement(str);
			if (child.isNull())
			{
				child = _doc.createElement(str);
				el.appendChild(child);
			}
			
			el = child;
		}
		
		// Set the value
		el.setAttribute("value", item->toString());
	}
	
	// Write XML
	QByteArray data = _doc.toByteArray();
	bool ok = (file.write(data) == data.size());
	if (!ok)
	{
		// Didn't write all the data
		error = file.errorString();
	}
	
	return ok;
}
