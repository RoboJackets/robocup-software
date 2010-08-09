#pragma once

#include <QFile>
#include <QStringList>
#include <vector>
#include <QDomDocument>

class QTreeWidget;
class QTreeWidgetItem;
class Configuration;

class ConfigItem
{
	public:
		ConfigItem(Configuration *tree, const QString &name);
		
		// A ConfigItem's name is a sequence of path segments separated by '/'.
		// The path is a list of these segments in order.
		const QStringList &path() const
		{
			return _path;
		}
		
		virtual QString toString() = 0;
		
		// Called by Configuration when the user changes the value
		virtual void setValue(const QString &str) = 0;
		
		void addToTree();
		
	protected:
		friend class Configuration;
		
		// Called by subclasses to update item text
		void valueChanged(const QString &str);
		
		QStringList _path;
		Configuration *_config;
		QTreeWidgetItem *_treeItem;
};

class ConfigInt: public ConfigItem
{
	public:
		ConfigInt(Configuration *tree, QString name, int value = 0);
		
		virtual QString toString();
		virtual void setValue(const QString &str);
		
		operator int() const
		{
			return _value;
		}
		
		int operator=(int x)
		{
			value(x);
			return x;
		}
		
		int value() const
		{
			return _value;
		}
		
		void value(int v)
		{
			_value = v;
			valueChanged(QString::number(v));
		}
	
	protected:
		int _value;
};

class ConfigDouble: public ConfigItem
{
	public:
		ConfigDouble(Configuration *tree, QString name, double value = 0);
		
		virtual QString toString();
		virtual void setValue(const QString &str);
		
		operator double() const
		{
			return _value;
		}
		
		double operator=(double x)
		{
			value(x);
			return x;
		}
		
		double value() const
		{
			return _value;
		}
		
		void value(double v)
		{
			_value = v;
			valueChanged(QString::number(v));
		}
	
	protected:
		double _value;
};

class ConfigVector: public ConfigItem
{
	public:
		ConfigVector(Configuration *tree, QString name):
			ConfigItem(tree, name)
		{
		}
		
		virtual void setElement(int i, const QString &str) = 0;
		virtual QString getElement(int i) = 0;
};

class ConfigVectorElement: public ConfigItem
{
	public:
		ConfigVectorElement(ConfigVector *vector, int index, Configuration *tree, QString name);
		
		virtual QString toString();
		virtual void setValue(const QString &str);

		// This is made public so the vector can change it
		void valueChanged(const QString &str)
		{
			ConfigItem::valueChanged(str);
		}

	protected:
		ConfigVector *_vector;
		int _index;
};

class ConfigFloatVector: public ConfigVector
{
	public:
		ConfigFloatVector(Configuration *tree, QString name);
		
		// The value of this item is the number of elements.  Each element is a child item.
		virtual QString toString();
		virtual void setValue(const QString &str);
		
		virtual void setElement(int i, const QString &str);
		virtual QString getElement(int i);
		
		const std::vector<float> &values() const
		{
			return _values;
		}
		
		void resize(unsigned int n);
		void set(unsigned int i, float value);
		
	protected:
		std::vector<float> _values;
		std::vector<ConfigVectorElement *> _items;
};

class Configuration: public QObject
{
	Q_OBJECT;
	
	public:
		Configuration();
		
		void tree(QTreeWidget *tree);
		
		QTreeWidget *tree() const
		{
			return _tree;
		}
		
		bool load(const QString &filename, QString &error);
		bool save(const QString &filename, QString &error);
		
	protected Q_SLOTS:
		void itemChanged(QTreeWidgetItem *item, int column);
		
	protected:
		friend class ConfigItem;
		
		// Called by ConfigItem's constructor
		void addItem(ConfigItem *item);
		
		void addToTree(ConfigItem *item);
		
		QList<ConfigItem *> _allItems;
		
		QTreeWidget *_tree;
		
		ConfigItem *configItem(QTreeWidgetItem *ti);
		
		QDomDocument _doc;
};
