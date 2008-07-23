#include "Test_Table_Model.hpp"
#include "Test_Interface.hpp"

Test_Table_Model::Test_Table_Model(Test_Interface *interface)
{
    _interface = interface;
    
    // Bits 2,1,0 are hall 3,2,1
    
    reference[0][0] = Test_Interface::Z;
    reference[0][1] = Test_Interface::Z;
    reference[0][2] = Test_Interface::Z;
    
    reference[1][0] = Test_Interface::H;
    reference[1][1] = Test_Interface::Z;
    reference[1][2] = Test_Interface::L;
    
    reference[2][0] = Test_Interface::L;
    reference[2][1] = Test_Interface::H;
    reference[2][2] = Test_Interface::Z;
    
    reference[3][0] = Test_Interface::Z;
    reference[3][1] = Test_Interface::H;
    reference[3][2] = Test_Interface::L;
    
    reference[4][0] = Test_Interface::Z;
    reference[4][1] = Test_Interface::L;
    reference[4][2] = Test_Interface::H;
    
    reference[5][0] = Test_Interface::H;
    reference[5][1] = Test_Interface::L;
    reference[5][2] = Test_Interface::Z;
    
    reference[6][0] = Test_Interface::L;
    reference[6][1] = Test_Interface::Z;
    reference[6][2] = Test_Interface::H;
    
    reference[7][0] = Test_Interface::Z;
    reference[7][1] = Test_Interface::Z;
    reference[7][2] = Test_Interface::Z;
    
    ccw = false;
    for (int i = 0; i < 8; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            motor_level[i][j] = Test_Interface::Z;
            motor_raw[i][j] = 0;
        }
    }
}

int Test_Table_Model::columnCount(const QModelIndex &parent) const
{
    return 6;
}

int Test_Table_Model::rowCount(const QModelIndex &parent) const
{
    return 8;
}

QModelIndex Test_Table_Model::parent(const QModelIndex &index) const
{
    return QModelIndex();
}

QVariant Test_Table_Model::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role == Qt::DisplayRole && orientation == Qt::Horizontal)
    {
        switch (section)
        {
            case 0: return "Hall A (1)";
            case 1: return "Hall B (2)";
            case 2: return "Hall C (3)";
            case 3: return "Motor A (1)";
            case 4: return "Motor B (2)";
            case 5: return "Motor C (3)";
        }
    }
    
    return QVariant();
}

QVariant Test_Table_Model::data(const QModelIndex &index, int role) const
{
    int row = index.row();
    int col = index.column();
    
    int m = col - 3;
    
    if (role == Qt::DisplayRole)
    {
        switch (col)
        {
            case 0:
            case 1:
            case 2:
                return (row & (1 << col)) ? 1 : 0;
            
            case 3:
            case 4:
            case 5:
            {
                QString str = level_name(motor_level[row][m]);
                
                Test_Interface::Output ref = get_ref(row, m);
                
                if (motor_level[row][m] == ref)
                {
                    return str;
                } else {
                    return str + " (" + level_name(ref) + ")";
                }
            }
        }
    } else if (role == Qt::BackgroundRole)
    {
        if (col >= 3 && col <= 5)
        {
            if (motor_level[row][m] == get_ref(row, m))
            {
                return Qt::darkGreen;
            } else {
                return Qt::darkRed;
            }
        }
    } else if (role == Qt::ForegroundRole)
    {
        if (col >= 3 && col <= 5)
        {
            return Qt::white;
        }
    }
    
    return QVariant();
}

void Test_Table_Model::refresh()
{
    dataChanged(index(0, 0), index(rowCount() - 1, columnCount() - 1));
}

Test_Interface::Output Test_Table_Model::get_ref(int input, int channel) const
{
    Test_Interface::Output ref = reference[input][channel];
    
    if (ccw)
    {
        switch (ref)
        {
            case Test_Interface::L: ref = Test_Interface::H; break;
            case Test_Interface::H: ref = Test_Interface::L; break;
        }
    }
    
    return ref;
}

QString Test_Table_Model::level_name(Test_Interface::Output value) const
{
    switch (value)
    {
        case Test_Interface::Z: return "Off";
        case Test_Interface::L: return "Low";
        case Test_Interface::H: return "High";
        default: return QString();
    }
}
