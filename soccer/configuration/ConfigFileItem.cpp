#include <configuration/ConfigFileItem.hpp>

#include <boost/foreach.hpp>


ConfigFileItem::ConfigFileItem(const QVector<QVariant> &data, ConfigFileItem *parent)
: _type(GROUP), _itemData(data), _parentItem(parent)
{
}

ConfigFileItem::ConfigFileItem(const QString& label, ConfigFileItem *parent)
: _type(LABEL), _parentItem(parent)
{
	_itemData << label << QVariant();
}

ConfigFileItem::ConfigFileItem(const QString& label, const float& value,
		ConfigFileItem *parent)
: _type(PARAM), _parentItem(parent)
{
	_itemData << label << value;
}

ConfigFileItem::~ConfigFileItem()
{
	qDeleteAll(_childItems);
}

void ConfigFileItem::appendChild(ConfigFileItem *item)
{
	_childItems.append(item);
}

ConfigFileItem *ConfigFileItem::child(int row)
{
	return _childItems.value(row);
}

int ConfigFileItem::childCount() const
{
	return _childItems.count();
}

int ConfigFileItem::columnCount() const
{
	return _itemData.count();
}

QVariant ConfigFileItem::data(int column) const
{
	return _itemData.value(column);
}

ConfigFileItem *ConfigFileItem::parent()
{
	return _parentItem;
}

int ConfigFileItem::row() const
{
	if (_parentItem)
		return _parentItem->_childItems.indexOf(const_cast<ConfigFileItem*>(this));

	return 0;
}

int ConfigFileItem::childNumber() const
{
	if (_parentItem)
		return _parentItem->_childItems.indexOf(const_cast<ConfigFileItem*>(this));

	return 0;
}

bool ConfigFileItem::insertChildren(int position, int count, int columns)
{
	if (position < 0 || position > _childItems.size())
		return false;

	for (int row = 0; row < count; ++row) {
		QVector<QVariant> data(columns);
		ConfigFileItem *item = new ConfigFileItem(data, this);
		_childItems.insert(position, item);
	}

	return true;
}

bool ConfigFileItem::insertColumns(int position, int columns)
{
	if (position < 0 || position > _itemData.size())
		return false;

	for (int column = 0; column < columns; ++column)
		_itemData.insert(position, QVariant());

	BOOST_FOREACH(ConfigFileItem *child, _childItems)
	child->insertColumns(position, columns);

	return true;
}

bool ConfigFileItem::removeChildren(int position, int count)
{
	if (position < 0 || position + count > _childItems.size())
		return false;

	for (int row = 0; row < count; ++row)
		delete _childItems.takeAt(position);

	return true;
}

bool ConfigFileItem::removeColumns(int position, int columns)
{
	if (position < 0 || position + columns > _itemData.size())
		return false;

	for (int column = 0; column < columns; ++column)
		_itemData.remove(position);

	BOOST_FOREACH (ConfigFileItem *child, _childItems) {
		child->removeColumns(position, columns);
	}

	return true;
}

bool ConfigFileItem::setData(int column, const QVariant &value)
{
	bool val_good;
	value.toFloat(&val_good);

	if (column != 1 || _type != PARAM || !val_good)
		return false;

	_itemData[column] = value;
	return true;
}
