#ifndef _ROBOT_TABLE_HPP_
#define _ROBOT_TABLE_HPP_

#include <QAbstractTableModel>
#include <QTimer>

class Command_Thread;

class Robot_Table_Model: public QAbstractTableModel
{
    Q_OBJECT;
    
public:
    // Time in milliseconds until status is shown in red.
    static const int Status_Timeout_ms = 400;
    
    Robot_Table_Model(Command_Thread *command_thread, QObject *parent = 0);
    
    virtual int columnCount(const QModelIndex &parent = QModelIndex()) const;
    virtual int rowCount(const QModelIndex &parent = QModelIndex()) const;
    virtual QModelIndex parent(const QModelIndex &index) const;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;
    virtual QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    
public Q_SLOTS:
    void refresh();
    
protected:
    static const char *column_name[];
    
    QTimer _refresh_timer;
    Command_Thread *_command_thread;
};

#endif // _ROBOT_TABLE_HPP_
