#include "Camera_Window.h"
#include "Camera_Window.moc"
#include "Transform_Setup.h"
#include "Config_File.h"
#include "Colorspace_Window.h"
#include "camera/Base.h"

#include "vision/Process.h"
#include "vision/Colorseg.h"
#include "vision/Distortion.h"
#include "vision/Transform.h"

//#include "vision/identification/Dot_ID.h"
//#include "vision/identification/Vector_ID.h"

#include <QFileDialog>
#include <QHeaderView>
#include <math.h>

using namespace std;
using namespace Eigen;

static const char *filter = "XML Configuration (*.xml);;All files (*)";

class Color_Table_Model: public QAbstractTableModel
{
public:
    Color_Table_Model(GL_Camera_View *view)
    {
        this->view = view;
    }

    virtual int rowCount(const QModelIndex &parent = QModelIndex()) const
    {
        return Vision::In_Use_Colors;
    }
    
    virtual int columnCount(const QModelIndex &parent = QModelIndex()) const
    {
        return 2;
    }
    
    virtual QVariant headerData(int n, Qt::Orientation orientation, int role = Qt::DisplayRole) const
    {
        if (role == Qt::DisplayRole)
        {
            if (orientation == Qt::Vertical)
            {
                return Vision::color_name[n];
            } else {
                switch (n)
                {
                    case 0:
                        return "Colorseg";
                    case 1:
                        return "Group";
                }
            }
        }
        
        return QVariant();
    }
    
    virtual QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const
    {
        int row = index.row();
        int col = index.column();
        
        switch (role)
        {
            case Qt::CheckStateRole:
                if (col == 0)
                {
                    return view->colorseg_flags[row] ? Qt::Checked : Qt::Unchecked;
                } else {
                    return view->group_flags[row] ? Qt::Checked : Qt::Unchecked;
                }
            default:
                return QVariant();
        }
    }
    
    virtual Qt::ItemFlags flags(const QModelIndex &index) const
    {
        return Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsEnabled;
    }
    
    virtual bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole)
    {
        if (role == Qt::CheckStateRole)
        {
            int row = index.row();
            int col = index.column();
            bool checked = value.toInt() == Qt::Checked ? true : false;
            
            if (col == 0)
            {
                view->colorseg_flags[row] = checked;
            } else {
                view->group_flags[row] = checked;
            }
            
            dataChanged(index, index);
            
            return true;
        } else {
            return false;
        }
    }

protected:
    GL_Camera_View *view;
};

////////

// Model for the transformation reference points table 
class Ref_Table_Model: public QAbstractTableModel
{
public:
	Ref_Table_Model(GL_Camera_View *view, Vision::Process *vision)
	{
		_view = view;
		_vision = vision;
	}
	
    virtual int rowCount(const QModelIndex &parent = QModelIndex()) const
    {
        return 3;
    }
    
    virtual int columnCount(const QModelIndex &parent = QModelIndex()) const
    {
        return 3;
    }
    
    virtual QVariant headerData(int n, Qt::Orientation orientation, int role = Qt::DisplayRole) const
    {
    	static const char *header_text[] = {
			"Point", "X", "Y"
    	};
    	
        if (role == Qt::DisplayRole && orientation == Qt::Horizontal)
        {
    		return header_text[n];
        }
        
        return QVariant();
    }
    
    virtual QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const
    {
    	if (role == Qt::DisplayRole)
    	{
    		int row = index.row();
    		
	    	switch (index.column())
	    	{
	    		case 0:			// Point
	    			return QVariant(QString::number(row));
				
	    		case 1:			// X
    				return QVariant(_vision->ball_transform->world_point[row].x);
	    		
	    		case 2:			// Y
    				return QVariant(_vision->ball_transform->world_point[row].y);
	    	}
    	}
    	
    	return QVariant();
    }
    
    virtual bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole)
    {
    	if (role == Qt::EditRole)
    	{
			int row = index.row();
    		switch (index.column())
    		{
    			case 1:		// X
    				_vision->ball_transform->world_point[row].x = value.toDouble();
    				_vision->robot_transform->world_point[row].x = value.toDouble();
    				break;

    			case 2:		// Y
    				_vision->ball_transform->world_point[row].y = value.toDouble();
    				_vision->robot_transform->world_point[row].y = value.toDouble();
    				break;
    		}
    		return true;
    	}
    	
    	return false;
    }
    
    virtual Qt::ItemFlags flags(const QModelIndex &index) const
    {
    	if (index.column() > 0)
    	{
    		return Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable;
    	} else {
    		return Qt::ItemIsSelectable | Qt::ItemIsEnabled;
    	}
    }
    
protected:
	GL_Camera_View *_view;
	Vision::Process *_vision;
};

////////

Camera_Window::Camera_Window(Config_File *config, QWidget *parent): QWidget(parent)
{
	_config_file = config;
    _colorspace_window = 0;
    _pattern_window = 0;
	
    ui.setupUi(this);
    
    // View setup
    Camera_Thread *ct = _config_file->camera_thread();
    ui.view->camera_window(this);
    ui.view->vision(ct->process);
    set_spanner_parameters();

    camera_tab_layout = new QVBoxLayout(ui.camera_tab);
    Camera::Base *cam = ct->camera();
    if (cam)
    {
        setWindowTitle(cam->name());
        
        // Camera config tab
        QWidget *config = cam->configuration();
        if (config)
        {
            camera_tab_layout->addWidget(config);
        }
    }
    
    ui.view->camera_thread = ct;
    connect(ct, SIGNAL(new_frame()), ui.view, SLOT(updateGL()));

    // Color to edit combobox
    ui.edit_color->addItem("<none>");
    for (int i = 0; i < Vision::In_Use_Colors; ++i)
    {
        ui.edit_color->addItem(Vision::color_name[i]);
        ui.spanner_color->addItem(Vision::color_name[i]);
    }
    ui.edit_color->setCurrentIndex(0);
    
    // Color table
    ui.color_table->setModel(new Color_Table_Model(ui.view));
    ui.color_table->resizeColumnsToContents();
    
    // Reference points table
    ui.ref_table->setModel(new Ref_Table_Model(ui.view, ct->process));
    ui.ref_table->resizeColumnsToContents();
    ui.ref_table->verticalHeader()->hide();
    
    // Checkboxes
    ui.show_camera->setChecked(ui.view->show_camera);
    ui.show_colorseg->setChecked(ui.view->show_colorseg);
    ui.show_groups->setChecked(ui.view->show_groups);
    ui.show_span_detail->setChecked(ui.view->show_span_detail);
    ui.show_reports->setChecked(ui.view->show_reports);
    
    // Parameters
    ui.robot_diameter->setValue(ui.view->robot_radius / 100);
}

void Camera_Window::on_edit_color_currentIndexChanged(int n)
{
    ui.view->edit_color = n - 1;
    
    if (n >= 1)
    {
        ui.spanner_color->setCurrentIndex(n - 1);
        
        if (_colorspace_window)
        {
            _colorspace_window->show_bin(n - 1);
        }
    }
}

void Camera_Window::on_spanner_color_currentIndexChanged(int n)
{
    ui.edit_color->setCurrentIndex(n + 1);
        
    if (_colorspace_window)
    {
        _colorspace_window->show_bin(n);
    }
    
    set_spanner_parameters();
}

void Camera_Window::on_show_camera_toggled(bool state)
{
    ui.view->show_camera = state;
    ui.view->update();
}

void Camera_Window::on_show_colorseg_toggled(bool state)
{
    ui.view->show_colorseg = state;
    ui.view->update();
}

void Camera_Window::on_show_groups_toggled(bool state)
{
    ui.view->show_groups = state;
    ui.view->update();
}

void Camera_Window::on_show_reports_toggled(bool state)
{
	ui.view->show_reports = state;
	ui.view->update();
}

void Camera_Window::on_show_span_detail_toggled(bool state)
{
    ui.view->show_span_detail = state;
    ui.view->update();
}

void Camera_Window::on_show_colorspace_clicked()
{
    if (!_colorspace_window)
    {
        _colorspace_window = new Colorspace_Window(this);
        _colorspace_window->colorseg(ui.view->camera_thread->process->colorseg);
        connect(ui.view, SIGNAL(colorseg_changed(QRgb)), _colorspace_window, SLOT(show_color(QRgb)));
        connect(_colorspace_window, SIGNAL(color_changed(int)),
            SLOT(on_spanner_color_currentIndexChanged(int)));
    }
    
    _colorspace_window->show();
}

void Camera_Window::on_view_mouse_moved(int x, int y)
{
    ui.status->setText(QString("(%1, %2)").arg(x).arg(y));

	Geometry::Point2d u = ui.view->vision()->distortion->undistort(Geometry::Point2d(x, y));
	Vector3f robot_pos = ui.view->vision()->robot_transform->matrix * Vector3f(u.x, u.y, 1);
	Vector3f ball_pos = ui.view->vision()->ball_transform->matrix * Vector3f(u.x, u.y, 1);
	
	ui.und_pos->setText(QString("%1, %2").arg(u.x, 0, 'f', 2).arg(u.y, 0, 'f', 2));
	ui.robot_pos->setText(QString("%1, %2").arg(robot_pos[0], 0, 'f', 2).arg(robot_pos[1], 0, 'f', 2));
	ui.ball_pos->setText(QString("%1, %2").arg(ball_pos[0], 0, 'f', 2).arg(ball_pos[1], 0, 'f', 2));
}

void Camera_Window::on_save_config_clicked()
{
    QString filename = _config_file->filename();
    QString dir;
    if (!filename.isNull())
    {
        dir = QFileInfo(filename).absolutePath();
    }
    
    QFileDialog dlg(this, "Save Configuration", dir, filter);
    dlg.setAcceptMode(QFileDialog::AcceptSave);
    dlg.selectFile(filename);
    if (dlg.exec() != QDialog::Accepted)
    {
        return;
    }
    
    filename = dlg.selectedFiles().first();
    if (!filename.isNull())
    {
        _config_file->save(filename);
    }
}

void Camera_Window::on_load_config_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, "Load Configuration", QString(), filter);
    if (!filename.isNull())
    {
    	_config_file->load(filename.toAscii().constData());
    }
}

void Camera_Window::on_clear_color_clicked()
{
    if (ui.view->edit_color >= 0)
    {
        ui.view->camera_thread->process->colorseg->clear_color(ui.view->edit_color);
        
        if (_colorspace_window)
        {
            _colorspace_window->update_view();
        }
    }
}

void Camera_Window::on_max_gap_valueChanged(int value)
{
	int color = ui.spanner_color->currentIndex();
    QMutexLocker ml(&ui.view->vision()->spanner[color]->mutex);
    ui.view->vision()->spanner[color]->max_gap = value;
}

void Camera_Window::on_min_span_pixels_valueChanged(int value)
{
	int color = ui.spanner_color->currentIndex();
    QMutexLocker ml(&ui.view->vision()->spanner[color]->mutex);
    ui.view->vision()->spanner[color]->min_span_pixels = value;
}

void Camera_Window::on_min_group_width_valueChanged(int value)
{
	int color = ui.spanner_color->currentIndex();
    QMutexLocker ml(&ui.view->vision()->spanner[color]->mutex);
    ui.view->vision()->spanner[color]->min_group_width = value;
}

void Camera_Window::on_max_group_width_valueChanged(int value)
{
	int color = ui.spanner_color->currentIndex();
    QMutexLocker ml(&ui.view->vision()->spanner[color]->mutex);
    ui.view->vision()->spanner[color]->max_group_width = value;
}

void Camera_Window::on_min_group_height_valueChanged(int value)
{
	int color = ui.spanner_color->currentIndex();
    QMutexLocker ml(&ui.view->vision()->spanner[color]->mutex);
    ui.view->vision()->spanner[color]->min_group_height = value;
}

void Camera_Window::on_max_group_height_valueChanged(int value)
{
	int color = ui.spanner_color->currentIndex();
    QMutexLocker ml(&ui.view->vision()->spanner[color]->mutex);
    ui.view->vision()->spanner[color]->max_group_height = value;
}

void Camera_Window::on_min_aspect_valueChanged(double value)
{
	int color = ui.spanner_color->currentIndex();
    QMutexLocker ml(&ui.view->vision()->spanner[color]->mutex);
    ui.view->vision()->spanner[color]->min_group_aspect = value;
}

void Camera_Window::on_max_aspect_valueChanged(double value)
{
	int color = ui.spanner_color->currentIndex();
    QMutexLocker ml(&ui.view->vision()->spanner[color]->mutex);
    ui.view->vision()->spanner[color]->max_group_aspect = value;
}

void Camera_Window::on_robot_diameter_valueChanged(double value)
{
    ui.view->robot_radius = value * 100;
    
    if (ui.view->vision())
    {
        ui.view->vision()->robot_radius(value / 2);
    }
}

void Camera_Window::set_spanner_parameters()
{
    int n = ui.spanner_color->currentIndex();
    if (n >= 0 && ui.view->vision())
    {
        Vision::Spanner *spanner = ui.view->vision()->spanner[n];
        if (spanner)
        {
            ui.max_gap->setValue(spanner->max_gap);
            ui.min_span_pixels->setValue(spanner->min_span_pixels);
            ui.min_group_width->setValue(spanner->min_group_width);
            ui.max_group_width->setValue(spanner->max_group_width);
            ui.min_group_height->setValue(spanner->min_group_height);
            ui.max_group_height->setValue(spanner->max_group_height);
            ui.min_aspect->setValue(spanner->min_group_aspect);
            ui.max_aspect->setValue(spanner->max_group_aspect);
        }
    }
}

void Camera_Window::edit_nothing()
{
    ui.edit_color->setCurrentIndex(0);
}
