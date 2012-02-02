#include "SimulatorWindow.hpp"
#include "SimRenderView.hpp"
#include "Physics/Environment.hpp"

#include <QAbstractTableModel>

static const QString columnNames[] = {
	"Visibility",
	"Ball Sensor",
	"Charger"
};

class RobotTableModel: public QAbstractTableModel
{
public:
	RobotTableModel()
	{
		_env = 0;
	}
	
	void env(Environment *value)
	{
		_env = value;
		layoutChanged();
	}
	
	Robot *robotForRow(int row) const
	{
		const Environment::RobotMap *map;
		if (row < _env->yellow().size())
		{
			map = &_env->yellow();
		} else {
			row -= _env->yellow().size();
			map = &_env->blue();
		}
		Environment::RobotMap::const_iterator i = map->begin();
		i += row;
		return *i;
	}
	
	virtual int columnCount(const QModelIndex &parent) const
	{
		return sizeof(columnNames) / sizeof(columnNames[0]);
	}
	
	virtual int rowCount(const QModelIndex &parent) const
	{
		if (_env)
		{
			return _env->blue().size() + _env->yellow().size();
		} else {
			return 0;
		}
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
		if (_env)
		{
			Robot *robot = robotForRow(index.row());
			
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
				if (index.row() < _env->yellow().size())
				{
					return QVariant(QColor(255, 255, 128));
				} else {
					return QVariant(QColor(128, 128, 255));
				}
			}
		}
		return QVariant();
	}
	
	virtual bool setData(const QModelIndex &index, const QVariant &value, int role)
	{
		Robot *robot = robotForRow(index.row());
		switch (index.column())
		{
			case 0:
				robot->visibility = value.toInt();
				return true;
			
			case 1:
				robot->ballSensorWorks = value.toBool();
				return true;
			
			case 2:
				robot->chargerWorks = value.toBool();
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
			} else if (_env)
			{
				return QVariant(robotForRow(section)->shell);
			}
		}
		return QVariant();
	}

private:
	Environment *_env;
};

////////////////////////////////////
// SimulatorWindow class
////////////////////////////////////
SimulatorWindow::SimulatorWindow(QWidget* parent):
	QMainWindow(parent)
{
	_env = 0;
	
	_ui.setupUi(this);
	
	// set up table
	_model = new RobotTableModel();
	_ui.robotTable->setModel(_model);
	_ui.robotTable->resizeColumnsToContents();

	// renderer setup
	_render = _ui.renderViewWidget;
}

void SimulatorWindow::env(Environment* value)
{
	_env = value;
	_model->env(value);
}

void SimulatorWindow::on_dropFrame_clicked()
{
	if (_env)
	{
		_env->dropFrame();
	}
}

void SimulatorWindow::on_ballVisibility_valueChanged(int value)
{
	if (_env)
	{
		_env->ballVisibility = value;
	}
}
