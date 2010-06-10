#include <QtGui>

#include <configuration/ConfigFileItem.hpp>
#include <configuration/ConfigFileModel.hpp>
#include <ConfigFileModel.moc>

#include <boost/foreach.hpp>

ConfigFileModel::ConfigFileModel(boost::shared_ptr<ConfigFile> config, QObject *parent)
 : QAbstractItemModel(parent),
   _default2008(config->defaultRobot2008()),
   _default2010(config->defaultRobot2010())
{
	QVector<QVariant> rootData;
	rootData << "Parameter" << "Value";
	_root = new ConfigFileItem(rootData);
	setupModelData();
}

ConfigFileModel::~ConfigFileModel()
{
	delete _root;
}

int ConfigFileModel::columnCount(const QModelIndex &parent) const
{
	if (parent.isValid())
		return static_cast<ConfigFileItem*>(parent.internalPointer())->columnCount();
	else
		return _root->columnCount();
}

QVariant ConfigFileModel::data(const QModelIndex &index, int role) const
{
	if (!index.isValid())
		return QVariant();

	if (role != Qt::DisplayRole && role != Qt::EditRole)
		return QVariant();

	ConfigFileItem *item = getItem(index);

	return item->data(index.column());
}

ConfigFileItem *ConfigFileModel::getItem(const QModelIndex &index) const
{
	if (index.isValid()) {
		ConfigFileItem *item = static_cast<ConfigFileItem*>(index.internalPointer());
		if (item) return item;
	}
	return _root;
}

Qt::ItemFlags ConfigFileModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
		return 0;

	ConfigFileItem *item = getItem(index);
	if (item->getType() == ConfigFileItem::PARAM && index.column() == 1)
		return Qt::ItemIsEditable | Qt::ItemIsEnabled | Qt::ItemIsSelectable;
	else
		return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

QVariant ConfigFileModel::headerData(int section, Qt::Orientation orientation,
		int role) const
{
	if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
		return _root->data(section);

	return QVariant();
}

QModelIndex ConfigFileModel::index(int row, int column, const QModelIndex &parent) const
{
	if (parent.isValid() && parent.column() != 0)
		return QModelIndex();

	ConfigFileItem *parentItem = getItem(parent);

	ConfigFileItem *childItem = parentItem->child(row);
	if (childItem)
		return createIndex(row, column, childItem);
	else
		return QModelIndex();
}

QModelIndex ConfigFileModel::parent(const QModelIndex &index) const
{
	if (!index.isValid())
		return QModelIndex();

	ConfigFileItem *childItem = getItem(index);
	ConfigFileItem *parentItem = childItem->parent();

	if (parentItem == _root)
		return QModelIndex();

	return createIndex(parentItem->childNumber(), 0, parentItem);
}

int ConfigFileModel::rowCount(const QModelIndex &parent) const
{
	ConfigFileItem *parentItem = getItem(parent);

	return parentItem->childCount();
}

void ConfigFileModel::setupModelData()
{
	// Create from data stored in default robots
	// one root for each robot

	// add an entry for each robot
	if (_default2008) {
		ConfigFileItem *robotRoot = addLabel(QString("Default 2008 Robot"), _root);
		setupRobotData(robotRoot, _default2008);
	}

	if (_default2010) {
		ConfigFileItem *robotRoot = addLabel(QString("Default 2010 Robot"), _root);
		setupRobotData(robotRoot, _default2010);
	}
}

void ConfigFileModel::setupRobotData(ConfigFileItem* robotRoot, ConfigFile::shared_robot config) {
	// create entries for data in a robot

	// dynamics models
	ConfigFileItem * deg0_dyn = addLabel(QString("Dynamics (0 Degrees)"), robotRoot);
	setupDynData(deg0_dyn, config->motion.deg0);
	ConfigFileItem * deg45_dyn = addLabel(QString("Dynamics (45 Degrees)"), robotRoot);
	setupDynData(deg45_dyn, config->motion.deg45);
	ConfigFileItem * rot_dyn = addLabel(QString("Dynamics (rotation)"), robotRoot);
	setupDynData(rot_dyn, config->motion.rotation);

	// PID
	ConfigFileItem * pid_angle = addLabel(QString("PID (rotation)"), robotRoot);
	addParam(QString("P"), config->motion.angle.p, pid_angle);
	addParam(QString("I"), config->motion.angle.i, pid_angle);
	addParam(QString("D"), config->motion.angle.d, pid_angle);


	// coefficients
	ConfigFileItem * coeffs = addLabel(QString("Output FIR Coeffs"), robotRoot);
	size_t idx = 0;
	BOOST_FOREACH(float coeff, config->motion.output_coeffs) {
		addParam(QString("A%1").arg(idx++), coeff, coeffs);
	}

}

void ConfigFileModel::setupDynData(ConfigFileItem* parent,
		const ConfigFile::Robot::Motion::Dynamics& model)
{
	addParam(QString("Acceleration"), model.acceleration, parent);
	addParam(QString("Deceleration"), model.deceleration, parent);
	addParam(QString("Velocity"), model.velocity, parent);
}

ConfigFileItem * ConfigFileModel::addLabel(const QString& label, ConfigFileItem * parent)
{
	ConfigFileItem *new_item = new ConfigFileItem(label, parent);
	parent->appendChild(new_item);
	return new_item;
}

ConfigFileItem * ConfigFileModel::addParam(const QString& label, const float& value, ConfigFileItem * parent)
{
	ConfigFileItem *new_item = new ConfigFileItem(label, value, parent);
	parent->appendChild(new_item);
	return new_item;
}

bool ConfigFileModel::insertColumns(int position, int columns, const QModelIndex &parent)
{
	bool success;

	beginInsertColumns(parent, position, position + columns - 1);
	success = _root->insertColumns(position, columns);
	endInsertColumns();

	return success;
}

bool ConfigFileModel::insertRows(int position, int rows, const QModelIndex &parent)
{
	ConfigFileItem *parentItem = getItem(parent);
	bool success;

	beginInsertRows(parent, position, position + rows - 1);
	success = parentItem->insertChildren(position, rows, _root->columnCount());
	endInsertRows();

	return success;
}

bool ConfigFileModel::removeColumns(int position, int columns, const QModelIndex &parent)
{
	bool success;

	beginRemoveColumns(parent, position, position + columns - 1);
	success = _root->removeColumns(position, columns);
	endRemoveColumns();

	if (_root->columnCount() == 0)
		removeRows(0, rowCount());

	return success;
}

bool ConfigFileModel::removeRows(int position, int rows, const QModelIndex &parent)
{
	ConfigFileItem *parentItem = getItem(parent);
	bool success = true;

	beginRemoveRows(parent, position, position + rows - 1);
	success = parentItem->removeChildren(position, rows);
	endRemoveRows();

	return success;
}

bool ConfigFileModel::setData(const QModelIndex &index, const QVariant &value,
		int role)
{
	if (role != Qt::EditRole)
		return false;

	// TODO: check that this is a value that makes sense

	ConfigFileItem *item = getItem(index);
	bool result = item->setData(index.column(), value);

//	if (result)
//		emit dataChanged(index, index);  // FIXME: need to make these?

	return result;
}

bool ConfigFileModel::setHeaderData(int section, Qt::Orientation orientation,
		const QVariant &value, int role)
{
	if (role != Qt::EditRole || orientation != Qt::Horizontal)
		return false;

	bool result = _root->setData(section, value);

//	if (result)
//		emit headerDataChanged(orientation, section, section); // FIXME: need to make these?

	return result;
}
