#pragma once

#include <QList>
#include <QVariant>
#include <QWidget>
#include <QAbstractItemModel>
#include <QStandardItem>

#include <framework/ConfigFile.hpp>

#include "ui_ConfigFileTab.h"

 class ConfigFileItem
 {
 public:
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


 private:
     QList<ConfigFileItem*> _childItems;
     QVector<QVariant> _itemData;
     ConfigFileItem * _parentItem;
 };


class ConfigFileModel : public QAbstractItemModel
{
	Q_OBJECT

public:
	ConfigFileModel(boost::shared_ptr<ConfigFile> config, QObject *parent = 0);
	virtual ~ConfigFileModel();

	virtual QVariant data(const QModelIndex &index, int role) const;
	virtual Qt::ItemFlags flags(const QModelIndex &index) const;
	virtual QVariant headerData(int section, Qt::Orientation orientation,
			int role = Qt::DisplayRole) const;
	virtual QModelIndex index(int row, int column,
			const QModelIndex &parent = QModelIndex()) const;
	virtual QModelIndex parent(const QModelIndex &index) const;
	virtual int rowCount(const QModelIndex &parent = QModelIndex()) const;
	virtual int columnCount(const QModelIndex &parent = QModelIndex()) const;

private:
	void setupModelData();
	void setupRobotData(ConfigFileItem* robotRoot, ConfigFile::shared_robot config);
	void setupDynData(ConfigFileItem* parent, const ConfigFile::Robot::Motion::Dynamics& model);

	// convenience functions for adding params and labels
	ConfigFileItem * addLabel(const QString& label, ConfigFileItem * parent);
	ConfigFileItem * addParam(const QString& label, const float& value, ConfigFileItem * parent);

	/** Store links to robot types */
	ConfigFile::shared_robot _default2008, _default2010;
	// TODO: add more details

	// tree components
	ConfigFileItem* _root;
};

class ConfigFileTab: public QWidget
{
	Q_OBJECT;

	public:
		ConfigFileTab(boost::shared_ptr<ConfigFile> config, QWidget *parent = 0);
		~ConfigFileTab(){}

		void save(QString filename);
		void load(QString filename);

	private Q_SLOTS:
		void on_loadConfig_clicked();
		void on_saveConfig_clicked();

		// TODO: add something for changes to table

	private:
		Ui_ConfigFileTab _ui;

		ConfigFileModel * _model;
};
