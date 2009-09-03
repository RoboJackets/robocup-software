#pragma once

#include "vision/Spanner.h"
#include "Image_Texture.h"

#include <Geometry2d/Point.hpp>
#include <QGLWidget>
#include <QGLFramebufferObject>
#include <QTime>
#include <vector>

#include "vision/Process.h"

class Camera_Thread;
class Camera_Window;
class Setup_Mode;
class Distortion_Setup;
class Transform_Setup;

namespace Vision
{
    class Calibration_Line;
}

class GL_Camera_View: public QGLWidget
{
    Q_OBJECT

public:
    GL_Camera_View(QWidget *parent);
    ~GL_Camera_View();

    void camera_window(Camera_Window *win);
    void vision(Vision::Process *vision);

    Vision::Process *vision() const { return _vision; }
    Geometry2d::Point last_mouse_pos() const { return _last_mouse_pos; }
    int frame_width() const { return _frame_width; }
    int frame_height() const { return _frame_height; }

    Camera_Thread *camera_thread;

    bool show_camera;
    bool show_colorseg;
    bool show_groups;
    bool show_span_detail;
    bool show_reports;
    std::vector<bool> colorseg_flags, group_flags;

    // View pixels per frame pixel
    float scale;

    // Offset in frame coordinates between the center of the frame and the center of the view
    float offset_x, offset_y;

    // Diameter of the robot dots
    float robot_radius;

    int edit_color;

    Transform_Setup *robot_transform_setup;
    Transform_Setup *ball_transform_setup;

Q_SIGNALS:
    // (x, y) is the position in the frame, not in the widget (not zoomed)
    void mouse_moved(int x, int y);

    void colorseg_changed(QRgb color);

public Q_SLOTS:
    void setup_ok();
    void setup_cancel();

protected Q_SLOTS:
    void setup_type_changed(int n);

protected:
	Camera_Window *_win;
    Vision::Process *_vision;
    int _frame_width, _frame_height;
    Image_Texture input_tex;
    QImage colorseg_image;
    Image_Texture colorseg_tex;
    QTime _last_frame_time;
    unsigned int last_frame_count;

    Distortion_Setup *distortion_setup;

    Setup_Mode *_setup;

    // Last mouse position in frame coordinates
    Geometry2d::Point _last_mouse_pos;

    void full_rect();
    void draw_spanner(const Vision::Spanner *spanner);

    // Converts view coordinates to frame coordinates
    Geometry2d::Point frame_position(QPoint pt);

    void mouse_edit_color(Geometry2d::Point &pos, Qt::MouseButtons buttons);

    virtual void initializeGL();
    virtual void resizeGL(int w, int h);
    virtual void paintGL();
    virtual void keyPressEvent(QKeyEvent *e);
    virtual void mousePressEvent(QMouseEvent *e);
    virtual void mouseDoubleClickEvent(QMouseEvent *e);
    virtual void mouseMoveEvent(QMouseEvent *e);
    virtual void wheelEvent(QWheelEvent *e);
};
