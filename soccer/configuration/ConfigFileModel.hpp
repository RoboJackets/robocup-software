#pragma once

 #include <QAbstractItemModel>
 #include <QModelIndex>
 #include <QVariant>


#include <framework/ConfigFile.hpp>

class ConfigFileItem;

class ConfigFileModel : public QAbstractItemModel
{
	Q_OBJECT;

public:
	ConfigFileModel(boost::shared_ptr<ConfigFile> config, QObject *parent = 0);
	~ConfigFileModel();

	QVariant data(const QModelIndex &index, int role) const;
	Qt::ItemFlags flags(const QModelIndex &index) const;
	QVariant headerData(int section, Qt::Orientation orientation,
			int role = Qt::DisplayRole) const;
	QModelIndex index(int row, int column,
			const QModelIndex &parent = QModelIndex()) const;
	QModelIndex parent(const QModelIndex &index) const;
	int rowCount(const QModelIndex &parent = QModelIndex()) const;
	int columnCount(const QModelIndex &parent = QModelIndex()) const;

	bool setData(const QModelIndex &index, const QVariant &value,
			int role = Qt::EditRole);
	bool setHeaderData(int section, Qt::Orientation orientation,
			const QVariant &value, int role = Qt::EditRole);
	bool insertColumns(int position, int columns,
			const QModelIndex &parent = QModelIndex());
	bool removeColumns(int position, int columns,
			const QModelIndex &parent = QModelIndex());
	bool insertRows(int position, int rows,
			const QModelIndex &parent = QModelIndex());
	bool removeRows(int position, int rows,
			const QModelIndex &parent = QModelIndex());

private:
	ConfigFileItem *getItem(const QModelIndex &index) const;

	/// general setup from a full config
	void setupModelData();
	void setupRobotData(ConfigFileItem* robotRoot, ConfigFile::shared_robot config);
	void setupDynData(ConfigFileItem* parent, const ConfigFile::Robot::Motion::Dynamics& model);
	void setupWorldModel(ConfigFileItem* parent, ConfigFile::shared_worldmodel config);

	/// change a specific parameter
	void changeParam(const QModelIndex &index, float value);
	void changeRobotParam(
			const QString& topLabel, const QString& groupLabel, const QString& paramLabel,
			float value);
	void changeWorldModelParam(const QString& groupLabel, const QString& paramLabel,
			float value);

	// convenience functions for adding params and labels
	ConfigFileItem * addLabel(const QString& label, ConfigFileItem * parent);
	ConfigFileItem * addParam(const QString& label, const float& value, ConfigFileItem * parent);

	/** Store links to robot types */
	ConfigFile::shared_robot _default2008, _default2010;

	/** Store links to the world model config data */
	ConfigFile::shared_worldmodel _worldmodel;

	ConfigFileItem* _root;
};
