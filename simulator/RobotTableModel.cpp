#include <QColor>

#include "RobotTableModel.hpp"
#include <physics/RobotBallController.hpp>

static const QString columnNames[] = {"Visibility", "Ball Sensor", "Charger"};

RobotTableModel::RobotTableModel(const Environment* env) {
    // load from the environment - before starting environment
    for (Environment::RobotMap::const_iterator it = env->yellow().begin();
         it != env->yellow().end(); ++it) {
        unsigned int shell = it.key();
        RobotID* r = new RobotID;
        r->ballSensorWorks = it.value()->getRobotController()->ballSensorWorks;
        r->chargerWorks = it.value()->getRobotController()->chargerWorks;
        r->visibility = it.value()->visibility;
        r->revision = it.value()->revision();
        r->shell = shell;
        _yellow.insert(shell, r);
    }

    for (Environment::RobotMap::ConstIterator it = env->blue().begin();
         it != env->blue().end(); ++it) {
        unsigned int shell = it.key();
        RobotID* r = new RobotID;
        r->ballSensorWorks = it.value()->getRobotController()->ballSensorWorks;
        r->chargerWorks = it.value()->getRobotController()->chargerWorks;
        r->visibility = it.value()->visibility;
        r->revision = it.value()->revision();
        r->shell = shell;
        _blue.insert(shell, r);
    }

    // configure layout
    layoutChanged();
}

// true = blue
bool RobotTableModel::teamForRow(int row) const {
    if (row < _yellow.size()) {
        return false;
    } else {
        return true;
    }
}

unsigned int RobotTableModel::shellForRow(int row) const {
    const RobotIDMap* map;
    if (!teamForRow(row)) {
        map = &_yellow;
    } else {
        row -= _yellow.size();
        map = &_blue;
    }
    RobotIDMap::const_iterator i = map->constBegin();
    i += row;
    return i.key();
}

const RobotTableModel::RobotID* RobotTableModel::robotForRow(int row) const {
    bool isblue = teamForRow(row);
    unsigned int shell = shellForRow(row);
    return (isblue) ? _blue.value(shell) : _yellow.value(shell);
}

RobotTableModel::RobotID* RobotTableModel::robotForRow(int row) {
    bool isblue = teamForRow(row);
    unsigned int shell = shellForRow(row);
    return (isblue) ? _blue.value(shell) : _yellow.value(shell);
}

int RobotTableModel::columnCount(const QModelIndex& parent) const {
    return sizeof(columnNames) / sizeof(columnNames[0]);
}

int RobotTableModel::rowCount(const QModelIndex& parent) const {
    return _blue.size() + _yellow.size();
}

Qt::ItemFlags RobotTableModel::flags(const QModelIndex& index) const {
    Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable;
    if (index.column() == 0) {
        flags |= Qt::ItemIsEditable;
    } else {
        flags |= Qt::ItemIsUserCheckable;
    }

    return flags;
}

QVariant RobotTableModel::data(const QModelIndex& index, int role) const {
    const RobotID* robot = robotForRow(index.row());

    if (index.column() == 0 &&
        (role == Qt::DisplayRole || role == Qt::EditRole)) {
        return QVariant(robot->visibility);
    } else if (index.column() == 1 && role == Qt::CheckStateRole) {
        return robot->ballSensorWorks ? Qt::Checked : Qt::Unchecked;
    } else if (index.column() == 2 && role == Qt::CheckStateRole) {
        return robot->chargerWorks ? Qt::Checked : Qt::Unchecked;
    } else if (role == Qt::BackgroundRole) {
        if (index.row() < _yellow.size()) {
            return QVariant(QColor(255, 255, 128));
        } else {
            return QVariant(QColor(128, 128, 255));
        }
    }
    return QVariant();
}

bool RobotTableModel::setData(const QModelIndex& index, const QVariant& value,
                              int role) {
    RobotID* robot = robotForRow(index.row());
    switch (index.column()) {
        case 0:
            robot->visibility = value.toInt();
            // emit setRobotVisibility(robot->visibility);
            return true;

        case 1:
            robot->ballSensorWorks = value.toBool();
            // emit setBallSensorWorks(robot->ballSensorWorks);
            return true;

        case 2:
            robot->chargerWorks = value.toBool();
            // emit setChargerWorks(robot->chargerWorks);
            return true;
    }

    return false;
}

QVariant RobotTableModel::headerData(int section, Qt::Orientation orientation,
                                     int role) const {
    if (role == Qt::DisplayRole) {
        if (orientation == Qt::Horizontal) {
            return columnNames[section];
        }
        return QVariant(robotForRow(section)->shell);
    }
    return QVariant();
}
