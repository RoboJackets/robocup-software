#ifndef _TEST_TABLE_MODEL_HPP_
#define _TEST_TABLE_MODEL_HPP_

#include <QAbstractTableModel>

#include "Test_Interface.hpp"

class Test_Table_Model: public QAbstractTableModel
{
public:
    Test_Table_Model(Test_Interface *interface);
    
    virtual int columnCount(const QModelIndex &parent = QModelIndex()) const;
    virtual int rowCount(const QModelIndex &parent = QModelIndex()) const;
    virtual QModelIndex parent(const QModelIndex &index) const;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;
    virtual QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    
    void refresh();
    
    bool ccw;
    float motor_raw[8][3];
    Test_Interface::Output motor_level[8][3];
    Test_Interface::Output reference[8][3];

protected:
    Test_Interface *_interface;
    
    Test_Interface::Output get_ref(int input, int channel) const;
    QString level_name(Test_Interface::Output value) const;
};

#endif // _TEST_TABLE_MODEL_HPP_
