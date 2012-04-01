#include "SimulatorWindow.hpp"
//#include "rendering/SimRenderView.hpp"
#include "Physics/Environment.hpp"

#include <boost/foreach.hpp>

#include <QAbstractTableModel>

static const QString columnNames[] = {
	"Visibility",
	"Ball Sensor",
	"Charger"
};

class RobotTableModel: public QAbstractTableModel
{
public:
	// Duplicate table to store robot info to avoid needing direct link to Environment
	struct RobotID {
		int visibility;
		bool ballSensorWorks;
		bool chargerWorks;
		unsigned int shell;
		Robot::RobotRevision revision;
	};

	typedef QMap<unsigned int, RobotID* > RobotIDMap;

	RobotTableModel(const Environment *env)
	{
		// load from the environment - before starting environment
		for (Environment::RobotMap::const_iterator it = env->yellow().begin();
				 it != env->yellow().end(); ++it) {
			unsigned int shell = it.key();
			RobotID * r = new RobotID;
			r->ballSensorWorks = it.value()->ballSensorWorks;
			r->chargerWorks = it.value()->chargerWorks;
			r->visibility = it.value()->visibility;
			r->revision = it.value()->revision();
			r->shell = shell;
			_yellow.insert(shell, r);
		}

		for (Environment::RobotMap::ConstIterator it = env->blue().begin();
				 it != env->blue().end(); ++it) {
			unsigned int shell = it.key();
			RobotID * r = new RobotID;
			r->ballSensorWorks = it.value()->ballSensorWorks;
			r->chargerWorks = it.value()->chargerWorks;
			r->visibility = it.value()->visibility;
			r->revision = it.value()->revision();
			r->shell = shell;
			_blue.insert(shell, r);
		}

		// configure layout
		layoutChanged();
	}
	
	// true = blue
	bool teamForRow(int row) const
	{
		if (row < _yellow.size())
		{
			return false;
		} else {
			return true;
		}
	}

	unsigned int shellForRow(int row) const
	{
		const RobotIDMap *map;
		if (!teamForRow(row))
		{
			map = &_yellow;
		} else {
			row -= _yellow.size();
			map = &_blue;
		}
		RobotIDMap::const_iterator i = map->constBegin();
		i += row;
		return i.key();
	}

	const RobotID * robotForRow(int row) const
	{
		bool isblue = teamForRow(row);
		unsigned int shell = shellForRow(row);
		return (isblue) ? _blue.value(shell) : _yellow.value(shell);
	}
	
	RobotID * robotForRow(int row)
	{
		bool isblue = teamForRow(row);
		unsigned int shell = shellForRow(row);
		return (isblue) ? _blue.value(shell) : _yellow.value(shell);
	}

	virtual int columnCount(const QModelIndex &parent) const
	{
		return sizeof(columnNames) / sizeof(columnNames[0]);
	}
	
	virtual int rowCount(const QModelIndex &parent) const
	{
		return _blue.size() + _yellow.size();
	}
	
	virtual Qt::ItemFlags flags(const QModelIndex &index) const
	{
		Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable;
		if (index.column() == 0)
		{
			flags |= Qt::ItemIsEditable;
		} else {
			flags |= Qt::ItemIsUserCheckable;
		}
		
		return flags;
	}
	
	virtual QVariant data(const QModelIndex &index, int role) const
	{
		const RobotID* robot = robotForRow(index.row());

		if (index.column() == 0 && (role == Qt::DisplayRole || role == Qt::EditRole))
		{
			return QVariant(robot->visibility);
		} else if (index.column() == 1 && role == Qt::CheckStateRole)
		{
			return robot->ballSensorWorks ? Qt::Checked : Qt::Unchecked;
		} else if (index.column() == 2 && role == Qt::CheckStateRole)
		{
			return robot->chargerWorks ? Qt::Checked : Qt::Unchecked;
		} else if (role == Qt::BackgroundRole)
		{
			if (index.row() < _yellow.size())
			{
				return QVariant(QColor(255, 255, 128));
			} else {
				return QVariant(QColor(128, 128, 255));
			}
		}
		return QVariant();
	}
	
	virtual bool setData(const QModelIndex &index, const QVariant &value, int role)
	{
		RobotID * robot = robotForRow(index.row());
		switch (index.column())
		{
			case 0:
				robot->visibility = value.toInt();
				emit setRobotVisibility(robot->visibility);
				return true;
			
			case 1:
				robot->ballSensorWorks = value.toBool();
				emit setBallSensorWorks(robot->ballSensorWorks);
				return true;
			
			case 2:
				robot->chargerWorks = value.toBool();
				emit setChargerWorks(robot->chargerWorks);
				return true;
		}
		
		return false;
	}
	
	virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const
	{
		if (role == Qt::DisplayRole)
		{
			if (orientation == Qt::Horizontal)
			{
				return columnNames[section];
			}
			return QVariant(robotForRow(section)->shell);
		}
		return QVariant();
	}

private:
	RobotIDMap _yellow, _blue;
};

////////////////////////////////////
// SimulatorWindow class
////////////////////////////////////
SimulatorWindow::SimulatorWindow(Environment * env, QWidget* parent):
	QMainWindow(parent)
{
	_ui.setupUi(this);

	// renderer setup
//	_render = _ui.renderViewWidget;

//	// connect renderer to simulator
//	connect(_env, SIGNAL(addNewRobot(bool,int)), _render, SLOT(addRobot(bool,int)));
//	connect(_env, SIGNAL(removeExistingRobot(bool,int)), _render, SLOT(removeRobot(bool,int)));
//	connect(_env, SIGNAL(setRobotPose(bool,int,QVector3D,qreal,QVector3D)),
//			    _render, SLOT(setRobotPose(bool,int,QVector3D,qreal,QVector3D)));

	// set up table
	_model = new RobotTableModel(env);
	_ui.robotTable->setModel(_model);
	_ui.robotTable->resizeColumnsToContents();
}

void SimulatorWindow::on_dropFrame_clicked()
{
	emit dropframe();
}

void SimulatorWindow::on_ballVisibility_valueChanged(int value)
{
	emit setBallVisibility(value);
}
