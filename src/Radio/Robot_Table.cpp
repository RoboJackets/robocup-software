#include "Robot_Table.hpp"
#include "Command_Thread.hpp"

#include <QComboBox>

const char *Robot_Table_Model::column_name[] =
{
    "Batt",
    "RSSI",
    "Ball"
};

Robot_Table_Model::Robot_Table_Model(Command_Thread *command_thread, QObject *parent): QAbstractTableModel(parent)
{
    _command_thread = command_thread;
    
    connect(&_refresh_timer, SIGNAL(timeout()), SLOT(refresh()));
    
    _refresh_timer.start(100);
}

int Robot_Table_Model::columnCount(const QModelIndex &parent) const
{
    return 3;
}

int Robot_Table_Model::rowCount(const QModelIndex &parent) const
{
    return Command_Thread::Num_Robots;
}

QModelIndex Robot_Table_Model::parent(const QModelIndex &index) const
{
    return QModelIndex();
}

QVariant Robot_Table_Model::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role == Qt::DisplayRole)
    {
        if (orientation == Qt::Horizontal)
        {
            return column_name[section];
        } else {
            return QString::number(section);
        }
    } else {
        return QVariant();
    }
}

QVariant Robot_Table_Model::data(const QModelIndex &index, int role) const
{
    int row = index.row();
    if (role == Qt::DisplayRole)
    {
        switch (index.column())
        {
            case 0:
            {
                QString str;
                str.sprintf("%.1fV", _command_thread->robot(row)->battery());
                return str;
            }
            
            case 1:
            {
                QString str;
                str.sprintf("%02x", _command_thread->robot(row)->rssi());
                return str;
            }
            
            case 2:
            {
                return _command_thread->robot(row)->ball() ? "yes" : "no";
            }
        }
    } else if (role == Qt::ForegroundRole)
    {
        if (_command_thread->robot(row)->update_time().msecsTo(QTime::currentTime()) > Status_Timeout_ms)
        {
            return Qt::white;
        }
    } else if (role == Qt::BackgroundRole)
    {
        if (_command_thread->robot(row)->update_time().msecsTo(QTime::currentTime()) > Status_Timeout_ms)
        {
            return QBrush(Qt::darkRed);
        }
    }
    
    return QVariant();
}

void Robot_Table_Model::refresh()
{
    dataChanged(index(0, 0), index(rowCount() - 1, columnCount() - 1));
}
