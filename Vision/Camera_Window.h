#pragma once

#include "ui_camera_window.h"
#include "Camera_Thread.h"

#include <QAbstractListModel>
#include <vector>

class QVBoxLayout;
class Config_File;
class Transform_Setup;
class Ref_Table_Model;
class Colorspace_Window;
class Pattern_Window;

namespace Vision
{
    class Colorseg;
    class Spanner;
};

class Camera_Window: public QWidget
{
    Q_OBJECT
    
public:
    Camera_Window(Config_File *config, QWidget *parent = 0);
    
    void set_spanner_parameters();
    void edit_nothing();
    
    GL_Camera_View *view() const { return ui.view; }

    Ui_Camera_Window ui;

protected Q_SLOTS:
    void on_edit_color_currentIndexChanged(int n);
    void on_spanner_color_currentIndexChanged(int n);
    void on_show_camera_toggled(bool state);
    void on_show_colorseg_toggled(bool state);
    void on_show_groups_toggled(bool state);
    void on_show_span_detail_toggled(bool state);
    void on_show_reports_toggled(bool state);
    void on_show_colorspace_clicked();
    void on_view_mouse_moved(int x, int y);
    void on_save_config_clicked();
    void on_load_config_clicked();
    void on_clear_color_clicked();
    void on_max_gap_valueChanged(int value);
    void on_min_span_pixels_valueChanged(int value);
    void on_min_group_width_valueChanged(int value);
    void on_max_group_width_valueChanged(int value);
    void on_min_group_height_valueChanged(int value);
    void on_max_group_height_valueChanged(int value);
    void on_min_aspect_valueChanged(double value);
    void on_max_aspect_valueChanged(double value);
    void on_robot_diameter_valueChanged(double value);

protected:
    int _zoom;
    Config_File *_config_file;
    QVBoxLayout *camera_tab_layout;
    Colorspace_Window *_colorspace_window;
    Pattern_Window *_pattern_window;
    
    QSize frame_size() const { return ui.view->camera_thread->frame_size(); }
};
