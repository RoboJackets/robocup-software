#include <QtGui>

#include <configuration/ConfigFileItem.hpp>
#include <configuration/ConfigFileModel.hpp>
// #include <ConfigFileModel.moc>

#include <iostream>
#include <boost/foreach.hpp>

using namespace std;

ConfigFileModel::ConfigFileModel(boost::shared_ptr<ConfigFile> config, QObject *parent)
 : QAbstractItemModel(parent),
   _default2008(config->defaultRobot2008()),
   _default2010(config->defaultRobot2010()),
   _worldmodel(config->worldModel)
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

	// add an entry for the world model
	if (_worldmodel) {
		ConfigFileItem *model_root = addLabel(QString("World Modeling"), _root);
		setupWorldModel(model_root, _worldmodel);
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

	// Wheel PID
	ConfigFileItem * pid_wheel = addLabel(QString("PID (wheels)"), robotRoot);
	addParam(QString("P"), config->motion.wheel.p, pid_wheel);
	addParam(QString("I"), config->motion.wheel.i, pid_wheel);
	addParam(QString("D"), config->motion.wheel.d, pid_wheel);

	// coefficients
	ConfigFileItem * coeffs = addLabel(QString("Output FIR Coeffs"), robotRoot);
	size_t idx = 0;
	BOOST_FOREACH(float coeff, config->motion.output_coeffs) {
		addParam(QString("%1").arg(idx++), coeff, coeffs);
	}

}

void ConfigFileModel::setupDynData(ConfigFileItem* parent,
		const ConfigFile::Robot::Motion::Dynamics& model)
{
	addParam(QString("Acceleration"), model.acceleration, parent);
	addParam(QString("Deceleration"), model.deceleration, parent);
	addParam(QString("Velocity"), model.velocity, parent);
}

void ConfigFileModel::setupWorldModel(ConfigFileItem* parent, ConfigFile::shared_worldmodel config) {
	// ABG filter
	ConfigFileItem * abg_root = addLabel(QString("ABG Robot Model"), parent);
	addParam(QString("alphaPos"), config->abgModelRobot.alphaPos, abg_root);
	addParam(QString("betaPos"), config->abgModelRobot.betaPos, abg_root);
	addParam(QString("gammaPos"), config->abgModelRobot.gammaPos, abg_root);
	addParam(QString("alphaAng"), config->abgModelRobot.alphaAng, abg_root);
	addParam(QString("betaAng"), config->abgModelRobot.betaAng, abg_root);
	addParam(QString("gammaAng"), config->abgModelRobot.gammaAng, abg_root);

	// Robot Kalman Filters
	ConfigFileItem * kalman_root = addLabel(QString("Kalman Robot Model"), parent);
	addParam(QString("covPosVel"), config->kalmanModelRobot.covPosVel, kalman_root);
	addParam(QString("covVelAcc"), config->kalmanModelRobot.covVelAcc, kalman_root);
	addParam(QString("covPosAcc"), config->kalmanModelRobot.covPosAcc, kalman_root);
	addParam(QString("covPos"), config->kalmanModelRobot.covPos, kalman_root);
	addParam(QString("covVel"), config->kalmanModelRobot.covVel, kalman_root);
	addParam(QString("covAcc"), config->kalmanModelRobot.covAcc, kalman_root);
	addParam(QString("measurementNoise"), config->kalmanModelRobot.measurementNoise, kalman_root);

	// RBPF parts
	ConfigFileItem * rbpf_roll_root = addLabel(QString("RBPF Ball Rolling"), parent);
	addParam(QString("processNoiseSqrdPos"), config->rbpfModelBallRolling.processNoiseSqrdPos, rbpf_roll_root);
	addParam(QString("processNoiseSqrdVel"), config->rbpfModelBallRolling.processNoiseSqrdVel, rbpf_roll_root);
	addParam(QString("processNoiseSqrdAcc"), config->rbpfModelBallRolling.processNoiseSqrdAcc, rbpf_roll_root);
	addParam(QString("measurementNoiseSqrd"), config->rbpfModelBallRolling.measurementNoiseSqrd, rbpf_roll_root);

	ConfigFileItem * rbpf_kicked_root = addLabel(QString("RBPF Ball Kicked"), parent);
	addParam(QString("processNoiseSqrdPos"), config->rbpfModelBallRolling.processNoiseSqrdPos, rbpf_kicked_root);
	addParam(QString("processNoiseSqrdVel"), config->rbpfModelBallRolling.processNoiseSqrdVel, rbpf_kicked_root);
	addParam(QString("processNoiseSqrdAcc"), config->rbpfModelBallRolling.processNoiseSqrdAcc, rbpf_kicked_root);
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

	ConfigFileItem *item = getItem(index);
	bool result = item->setData(index.column(), value);

	// change the value in the underlying structure
	if (result) {
		changeParam(index, value.toFloat());
	}

	return result;
}

bool ConfigFileModel::setHeaderData(int section, Qt::Orientation orientation,
		const QVariant &value, int role)
{
	if (role != Qt::EditRole || orientation != Qt::Horizontal)
		return false;

	bool result = _root->setData(section, value);

	return result;
}

void ConfigFileModel::changeParam(const QModelIndex &index, float value) {
	ConfigFileItem* param = getItem(index);
	QString paramLabel = param->getLabel();
	QString groupLabel = param->parent()->getLabel();
	QString topLabel = param->parent()->parent()->getLabel();

	if (topLabel.contains(QString("Robot")))
		changeRobotParam(topLabel, groupLabel, paramLabel, value);
	else if (topLabel.contains(QString("World")))
		changeWorldModelParam(groupLabel, paramLabel, value);
}

void ConfigFileModel::changeRobotParam(
		const QString& robotLabel, const QString& groupLabel, const QString& paramLabel,
		float value) {
	bool verbose = false;

	// determine the robot
	ConfigFile::shared_robot cur_robot;
	if (robotLabel.contains(QString("2008"))) {
		cur_robot = _default2008;
		if (verbose) cout << "changeParam: changing default2008" << endl;
	} else if (robotLabel.contains(QString("2010"))) {
		cur_robot = _default2010;
		if (verbose) cout << "changeParam: changing default2010" << endl;
	}

	// handle individual components
	if (groupLabel.contains(tr("PID"))) {
		if (paramLabel.contains(tr("P")))
			cur_robot->motion.angle.p = value;
		else if (paramLabel.contains(tr("I")))
			cur_robot->motion.angle.i = value;
		else if (paramLabel.contains(tr("D")))
			cur_robot->motion.angle.d = value;
		else
			cout << "IN PID: did not get correct paramLabel!" << endl;
	}
	else if (groupLabel.contains(tr("Dynamics"))) {
		ConfigFile::Robot::Motion::Dynamics * cur_dyn = 0;
		if (groupLabel.contains(tr("0 Degrees")))
			cur_dyn = &(cur_robot->motion.deg0);
		else if (groupLabel.contains(tr("45 Degrees")))
			cur_dyn = &(cur_robot->motion.deg45);
		else if (groupLabel.contains(tr("rotation")))
			cur_dyn = &(cur_robot->motion.rotation);
		else
			cout << "IN DYNAMICS: did not get correct group Label!" << endl;

		if (paramLabel.contains(tr("Acceleration")))
			cur_dyn->acceleration = value;
		else if (paramLabel.contains(tr("Velocity")))
			cur_dyn->velocity = value;
		else if (paramLabel.contains(tr("Deceleration")))
			cur_dyn->deceleration = value;
	}
	else if (groupLabel.contains(tr("Output"))) {
		bool val_good;
		size_t idx = paramLabel.toUInt(&val_good);
		if (!val_good)
			cout << "IN changeParam: bad index for paramLabel in coeffs!" << endl;
		cur_robot->motion.output_coeffs[idx] = value;
	} else {
		cout << "IN changeParam: Did not get a valid groupLabel!" << endl;
	}
}

void ConfigFileModel::changeWorldModelParam(const QString& groupLabel, const QString& paramLabel, float value) {

	if (groupLabel.contains(QString("ABG"))) {
		if (paramLabel.contains(QString("alphaPos")))
			_worldmodel->abgModelRobot.alphaPos = value;
		else if (paramLabel.contains(QString("betaPos")))
			_worldmodel->abgModelRobot.betaPos = value;
		else if (paramLabel.contains(QString("gammaPos")))
			_worldmodel->abgModelRobot.gammaPos = value;
		else if (paramLabel.contains(QString("alphaAng")))
			_worldmodel->abgModelRobot.alphaAng = value;
		else if (paramLabel.contains(QString("betaAng")))
			_worldmodel->abgModelRobot.betaAng = value;
		else if (paramLabel.contains(QString("gammaAng")))
			_worldmodel->abgModelRobot.gammaAng = value;

	} else if (groupLabel.contains(QString("Kalman"))) {
		if (paramLabel.contains(QString("covPosVel")))
			_worldmodel->kalmanModelRobot.covPosVel = value;
		else if (paramLabel.contains(QString("covVelAcc")))
			_worldmodel->kalmanModelRobot.covVelAcc = value;
		else if (paramLabel.contains(QString("covPosAcc")))
			_worldmodel->kalmanModelRobot.covPosAcc = value;
		else if (paramLabel.contains(QString("covPos")))
			_worldmodel->kalmanModelRobot.covPos = value;
		else if (paramLabel.contains(QString("covVel")))
			_worldmodel->kalmanModelRobot.covVel = value;
		else if (paramLabel.contains(QString("covAcc")))
			_worldmodel->kalmanModelRobot.covAcc = value;
		else if (paramLabel.contains(QString("measurementNoise")))
			_worldmodel->kalmanModelRobot.measurementNoise = value;

	} else if (groupLabel.contains(QString("Rolling"))) {
		if (paramLabel.contains(QString("processNoiseSqrdPos")))
			_worldmodel->rbpfModelBallRolling.processNoiseSqrdPos = value;
		else if (paramLabel.contains(QString("processNoiseSqrdVel")))
			_worldmodel->rbpfModelBallRolling.processNoiseSqrdVel = value;
		else if (paramLabel.contains(QString("processNoiseSqrdAcc")))
			_worldmodel->rbpfModelBallRolling.processNoiseSqrdAcc = value;
		else if (paramLabel.contains(QString("measurementNoiseSqrd")))
			_worldmodel->rbpfModelBallRolling.measurementNoiseSqrd = value;

	} else if (groupLabel.contains(QString("Kicked"))) {
		if (paramLabel.contains(QString("processNoiseSqrdPos")))
			_worldmodel->rbpfModelBallKicked.processNoiseSqrdPos = value;
		else if (paramLabel.contains(QString("processNoiseSqrdVel")))
			_worldmodel->rbpfModelBallKicked.processNoiseSqrdVel = value;
		else if (paramLabel.contains(QString("processNoiseSqrdAcc")))
			_worldmodel->rbpfModelBallKicked.processNoiseSqrdAcc = value;
		else if (paramLabel.contains(QString("measurementNoiseSqrd")))
			_worldmodel->rbpfModelBallKicked.measurementNoiseSqrd = value;
	}

}
