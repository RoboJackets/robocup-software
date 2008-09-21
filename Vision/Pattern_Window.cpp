#include "Pattern_Window.h"
#include "Pattern_Window.moc"

#include <QFileDialog>
#include <QMenu>
#include <QHeaderView>

#include <boost/foreach.hpp>

static const char *filter = "Images (*.png *.jpg);;All files (*)";

using namespace std;
using namespace Vision;

class Vector_Model: public QAbstractTableModel
{
public:
    Vector_Model(Vector_Pattern *p)
    {
        _pattern = p;
    }
    
    virtual int rowCount(const QModelIndex &parent = QModelIndex()) const
    {
        return _pattern->pairs.size();
    }
    
    virtual int columnCount(const QModelIndex &parent = QModelIndex()) const
    {
        return 3;
    }
    
    virtual QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const
    {
        int row = index.row();
        int col = index.column();
        
        if (role == Qt::DisplayRole)
        {
            switch (col)
            {
                case 0:
                    return _pattern->pairs[row].i0;
                    
                case 1:
                    return _pattern->pairs[row].i1;
                    
                case 2:
                    return _pattern->pairs[row].angle;
                    
                default:
                    return QVariant();
            }
        } else {
            return QVariant();
        }
    }
    
    void add(const Vector_Pattern::Pair &p)
    {
        beginInsertRows(QModelIndex(), rowCount(), rowCount());
        _pattern->pairs.push_back(p);
        endInsertRows();
    }
    
    void remove(const QModelIndex &index)
    {
        beginRemoveRows(QModelIndex(), index.row(), index.row());
        _pattern->pairs.erase(_pattern->pairs.begin() + index.row());
        endRemoveRows();
    }
    
    void refresh()
    {
        printf("have %d\n", (int)_pattern->pairs.size());
        dataChanged(index(0, 0), index(rowCount(), columnCount()));
    }
    
protected:
    Vector_Pattern *_pattern;
};

////////

Pattern_Window::Pattern_Window(QWidget *parent): QDialog(parent)
{
    ui.setupUi(this);
    
    ui.view->pattern = &_pattern;
    
    _model = new Vector_Model(&_pattern);
    ui.vectors->setModel(_model);
    ui.vectors->horizontalHeader()->hide();
    ui.vectors->verticalHeader()->hide();

    ui.view->load_image("../butterfly.png");
    update_offsets();
}

void Pattern_Window::on_load_image_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, "Load Image", QString(), filter);
    if (!filename.isNull())
    {
        ui.view->load_image(filename);
    }
}

void Pattern_Window::on_vectors_customContextMenuRequested(const QPoint &pt)
{
    QMenu menu;
    
    QAction *remove = new QAction("Remove", this);
    menu.addAction(remove);
    QAction *a = menu.exec(QCursor::pos());
    
    if (a == remove)
    {
        QModelIndex index = ui.vectors->indexAt(pt);
        if (index.isValid())
        {
            _model->remove(index);
            ui.view->update();
            update_offsets();
        }
    }
}

void Pattern_Window::on_view_vector_clicked(int i0, int i1)
{
    const vector<Dot_ID::Dot> &dots = ui.view->dots();
    Geometry::Point2d v = dots[i1].group->raw_center - dots[i0].group->raw_center;
    
    float a = fix_angle(90.0 - v.angle() * 180.0 / M_PI);
    _model->add(Vector_Pattern::Pair(i0, i1, a));
    ui.vectors->resizeColumnsToContents();
    ui.view->update();
    update_offsets();
}

void Pattern_Window::update_offsets()
{
    Geometry::Point2d offset = ui.view->center_group()->raw_center - ui.view->dot_id()->center();
    ui.center->setText(QString("%1, %2").arg(offset.x).arg(offset.y));

    const vector<Dot_ID::Dot> &dots = ui.view->dots();
    Geometry::Point2d facing;
    BOOST_FOREACH(const Vector_Pattern::Pair &pair, _pattern.pairs)
    {
        Geometry::Point2d v = dots[pair.i1].group->center - dots[pair.i0].group->center;
        v.rotate(Geometry::Point2d(), pair.angle);
        facing += v;
    }
    float angle = fix_angle(90.0 - facing.angle() * 180.0 / M_PI);
    ui.angle->setText(QString::number(angle));
}
