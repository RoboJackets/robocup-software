#include <ConfigFileTab.hpp>
#include <ConfigFileTab.moc>

#include <boost/foreach.hpp>

///////// Item

ConfigFileItem::ConfigFileItem(const QVector<QVariant> &data, ConfigFileItem *parent)
: _itemData(data), _parentItem(parent)
{
}

ConfigFileItem::ConfigFileItem(const QString& label, ConfigFileItem *parent)
: _parentItem(parent)
{
	_itemData << label << QVariant();
}

ConfigFileItem::ConfigFileItem(const QString& label, const float& value,
		ConfigFileItem *parent)
: _parentItem(parent)
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

	BOOST_FOREACH (ConfigFileItem *child, _childItems)
		child->removeColumns(position, columns);

	return true;
}

bool ConfigFileItem::setData(int column, const QVariant &value)
{
	if (column < 0 || column >= _itemData.size())
		return false;

	_itemData[column] = value;
	return true;
}

////////////// Model

ConfigFileModel::ConfigFileModel(boost::shared_ptr<ConfigFile> config, QObject *parent)
{
	_default2008 = config->defaultRobot2008();
	_default2010 = config->defaultRobot2010();
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

	if (role != Qt::DisplayRole)
		return QVariant();

	ConfigFileItem *item = static_cast<ConfigFileItem*>(index.internalPointer());

	return item->data(index.column());
}

Qt::ItemFlags ConfigFileModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
		return 0;

	return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

QVariant ConfigFileModel::headerData(int section, Qt::Orientation orientation,
		int role) const
{
	if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
		return _root->data(section);

	return QVariant();
}

QModelIndex ConfigFileModel::index(int row, int column, const QModelIndex &parent)
const
{
	if (!hasIndex(row, column, parent))
		return QModelIndex();

	ConfigFileItem *parentItem;

	if (!parent.isValid())
		parentItem = _root;
	else
		parentItem = static_cast<ConfigFileItem*>(parent.internalPointer());

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

	ConfigFileItem *childItem = static_cast<ConfigFileItem*>(index.internalPointer());
	ConfigFileItem *parentItem = childItem->parent();

	if (parentItem == _root)
		return QModelIndex();

	return createIndex(parentItem->row(), 0, parentItem);
}

int ConfigFileModel::rowCount(const QModelIndex &parent) const
{
	ConfigFileItem *parentItem;
	if (parent.column() > 0)
		return 0;

	if (!parent.isValid())
		parentItem = _root;
	else
		parentItem = static_cast<ConfigFileItem*>(parent.internalPointer());

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

///////////

ConfigFileTab::ConfigFileTab(boost::shared_ptr<ConfigFile> config, QWidget *parent)
: QWidget(parent), _model(new ConfigFileModel(config, this))
{
	_ui.setupUi(this);

	_ui.paramTree->setModel(_model);
}

void ConfigFileTab::save(QString filename) {

}

void ConfigFileTab::load(QString filename) {

}

void ConfigFileTab::on_loadConfig_clicked() {
	// TODO: add load code here
}

void ConfigFileTab::on_saveConfig_clicked() {
	// TODO: add save code here
}




