#pragma once

#include <QList>
#include <QVector>
#include <QVariant>

class ConfigFileItem
{
public:
	// flags to determine the type
	typedef enum {
		GROUP,
		LABEL,
		PARAM
	} ItemType;

	// general constructor
	ConfigFileItem(const QVector<QVariant> &data, ConfigFileItem *parent = 0);

	// one and two element constructors
	ConfigFileItem(const QString& label, ConfigFileItem *parent = 0);
	ConfigFileItem(const QString& label, const float& value, ConfigFileItem *parent = 0);

	virtual ~ConfigFileItem();

	void appendChild(ConfigFileItem *child);

	ConfigFileItem *child(int row);
	int childCount() const;
	int columnCount() const;
	QVariant data(int column) const;
	int row() const;
	ConfigFileItem *parent();

	bool insertChildren(int position, int count, int columns);
	bool insertColumns(int position, int columns);
	bool removeChildren(int position, int count);
	bool removeColumns(int position, int columns);
	int childNumber() const;
	bool setData(int column, const QVariant &value);

	ItemType getType() const {
		return _type;
	}

private:
	ItemType _type;

	QList<ConfigFileItem*> _childItems;
	QVector<QVariant> _itemData;
	ConfigFileItem * _parentItem;
};
