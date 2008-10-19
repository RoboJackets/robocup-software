#include "GL_Camera_View.h"
#include "GL_Camera_View.moc"
#include "Camera_Window.h"
#include "Camera_Thread.h"
#include "Distortion_Setup.h"
#include "Transform_Setup.h"
#include "main.hpp"

#include "vision/Process.h"
#include "vision/Colorseg.h"
#include "vision/Spanner.h"
#include "vision/Transform.h"

#include <QMouseEvent>
#include <QMessageBox>
#include <vector>
#include <stdexcept>
#include <math.h>
#include <sys/time.h>
#include <GL/glut.h>
#include <boost/foreach.hpp>

using namespace std;

static uint8_t color_table[8][4] =
{
    {255, 128, 0, 255},     // 0 - Orange
    {64, 64, 255, 255},     // 1 - Blue
    {255, 255, 0, 255},     // 2 - Yellow
    {0, 255, 0, 255},       // 3 - Green
    {255, 0, 0, 255},       // 4 - Pink
    {255, 255, 255, 255},   // 5 - White
    {0, 0, 0, 255},         // 6 - Unused
    {0, 0, 0, 255},         // 7 - Unused
};

GL_Camera_View::GL_Camera_View(QWidget *parent): QGLWidget(parent),
    colorseg_flags(8, true), group_flags(8, true),
    colorseg_tex(4, GL_RGBA)
{
    scale = 1;
    offset_x = 0;
    offset_y = 0;
    edit_color = -1;
    _frame_width = _frame_height = 0;
    last_frame_count = 0;
    _vision = 0;
    camera_thread = 0;
    robot_radius = 18;
    _setup = 0;

    distortion_setup = new Distortion_Setup(this);
    robot_transform_setup = new Transform_Setup(this);
    ball_transform_setup = new Transform_Setup(this);

    // Colorseg_Window uses these to set the checkboxes when the window is created.
    show_camera = true;
    show_colorseg = false;
    show_span_detail = false;
    show_groups = false;
    show_reports = true;

    setMouseTracking(true);
    setFocusPolicy(Qt::WheelFocus);
    setCursor(Qt::ArrowCursor);
}

GL_Camera_View::~GL_Camera_View()
{
	delete distortion_setup;
	delete robot_transform_setup;
	delete ball_transform_setup;
}

void GL_Camera_View::camera_window(Camera_Window *win)
{
	_win = win;
	connect(_win->ui.setup_type, SIGNAL(currentIndexChanged(int)), SLOT(setup_type_changed(int)));
	connect(_win->ui.setup_ok, SIGNAL(clicked()), SLOT(setup_ok()));
	connect(_win->ui.setup_cancel, SIGNAL(clicked()), SLOT(setup_cancel()));
}

void GL_Camera_View::vision(Vision::Process *vision)
{
	_vision = vision;

	ball_transform_setup->transform = _vision->ball_transform;
	robot_transform_setup->transform = _vision->robot_transform;
}

void GL_Camera_View::setup_type_changed(int n)
{
	if (_setup)
	{
		_setup->cancel();
		_setup = 0;
	}

	switch (n)
	{
		case 1:			// Distortion
			_win->ui.setup_ok->setEnabled(true);
			_setup = distortion_setup;
			break;

		case 2:			// Robot transform
			_setup = robot_transform_setup;
			break;

		case 3:			// Ball transform
			_setup = ball_transform_setup;
			break;

		default:		// None
			_win->ui.setup_ok->setEnabled(false);
			_win->ui.setup_cancel->setEnabled(false);
			break;
	}

	if (_setup)
	{
		_win->ui.setup_cancel->setEnabled(true);
		_setup->start();
	}
}

void GL_Camera_View::setup_ok()
{
	if (_setup)
	{
		_setup->ok();
		_setup = 0;
	}

	_win->ui.setup_type->setCurrentIndex(0);
	_win->ui.setup_ok->setEnabled(false);
	_win->ui.setup_cancel->setEnabled(false);
}

void GL_Camera_View::setup_cancel()
{
	if (_setup)
	{
		_setup->cancel();
		_setup = 0;
	}

	_win->ui.setup_type->setCurrentIndex(0);
	_win->ui.setup_ok->setEnabled(false);
	_win->ui.setup_cancel->setEnabled(false);
}

void GL_Camera_View::full_rect()
{
    glBegin(GL_POLYGON);
        glTexCoord2f(0, 0);
        glVertex2i(0, 0);
		
        glTexCoord2f(_frame_width, 0);
        glVertex2i(_frame_width, 0);

        glTexCoord2f(_frame_width, _frame_height);
        glVertex2i(_frame_width, _frame_height);

		glTexCoord2f(0, _frame_height);
        glVertex2i(0, _frame_height);
    glEnd();
}

void circle_verts(float x, float y, float r)
{
    for (int i = 0; i < 360; i += 20)
    {
        glVertex2f(x + r * cos(i * M_PI / 180), y + r * sin(i * M_PI / 180));
    }
}

void GL_Camera_View::draw_spanner(const Vision::Spanner *spanner)
{
    uint8_t *color = color_table[spanner->color()];

    if (show_span_detail)
    {
        // Spans
        glColor3ubv(color);
        glBegin(GL_LINES);
        BOOST_FOREACH(Vision::Span *span, spanner->spans())
        {
            glVertex2f(span->x1 + 0.5, span->y + 0.5);
            glVertex2f(span->x2 + 0.5, span->y + 0.5);
        }
        glEnd();
    }

    BOOST_FOREACH(const Vision::Group *group, spanner->groups())
    {
        glColor3ubv(color);

        if (show_groups)
        {
            // Bounding box
            glBegin(GL_LINE_LOOP);
                glVertex2i(group->min_x, group->min_y);
                glVertex2i(group->max_x + 1, group->min_y);
                glVertex2i(group->max_x + 1, group->max_y + 1);
                glVertex2i(group->min_x, group->max_y + 1);
            glEnd();

            if (group->id >= 0)
            {
                glColor3ub(255, 255, 255);
                glPushMatrix();
                char ch = group->id + '0';
                glTranslatef(group->raw_center.x, group->raw_center.y, 0);
                glScalef(0.002 * robot_radius, -0.002 * robot_radius, 1);

                int w = glutStrokeWidth(GLUT_STROKE_ROMAN, ch);
                glTranslatef(-w / 2, -60, 0);
                glutStrokeCharacter(GLUT_STROKE_ROMAN, ch);
                glPopMatrix();
            }
        }
    }
}

Geometry::Point2d GL_Camera_View::frame_position(QPoint pt)
{
    float x = ((float)pt.x() - width() / 2) / scale + offset_x + _frame_width / 2;
    float y = ((float)pt.y() - height() / 2) / scale + offset_y + _frame_height / 2;

    return Geometry::Point2d(x, y);
}

void GL_Camera_View::initializeGL()
{
    input_tex.generate();
    colorseg_tex.generate();

	//http://www.opengl.org/registry/specs/ARB/texture_rectangle.txt
	//all textures must be created with this as a texture target
    glEnable(GL_TEXTURE_RECTANGLE_ARB);

    // Blank texture
    vector<uint8_t> blank(4 * 4, 255);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
    glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, 4, 2, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, &blank[0]);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void GL_Camera_View::resizeGL(int w, int h)
{
}

void GL_Camera_View::paintGL()
{
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (GL_NO_ERROR != glGetError()) printf("GL error\n");

    // Get the camera frame.
    // full_rect also needs frame size to be set.
    camera_thread->frame_texture(input_tex);

    QSize frame_size = camera_thread->frame_size();
    int w = frame_size.width();
    int h = frame_size.height();

    if (w <= 0 || h <= 0)
    {
        return;
    }

    if (w != _frame_width || h != _frame_height)
    {
        printf("Frame size %dx%d\n", w, h);
        _frame_width = w;
        _frame_height = h;

        colorseg_image = QImage(w, h, QImage::Format_RGB32);
    }

    glViewport(0, 0, width(), height());
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-width() / 2, width() / 2, height() / 2, -height() / 2, 0, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glScalef(scale, scale, 1);
    glTranslatef(-offset_x - _frame_width / 2, -offset_y - _frame_height / 2, 0);

    glColor3ub(255, 255, 255);

    // Camera input
    if (show_camera)
    {
        // Draw the camera frame
        glDisable(GL_BLEND);
        input_tex.use();
        full_rect();
    } else {
        glClear(GL_COLOR_BUFFER_BIT);
    }

    glEnable(GL_BLEND);

    // Colorseg output
    if (show_colorseg)
    {
        if (show_groups || show_span_detail)
        {
            // Dim colorseg so groups are easier to see
            glColor4ub(255, 255, 255, 128);
        }

        const QImage &cs = _vision->colorseg->output();
        if (!cs.isNull())
        {
            int num_pixels = _frame_width * _frame_height;
            const uint8_t *in = cs.bits();
            uint32_t *out = (uint32_t *)colorseg_image.bits();
            for (int i = 0; i < num_pixels; ++i)
            {
                uint8_t seg = *in++;
                uint32_t rgb = 0;
                for (int j = 0; j < 8; ++j)
                {
                    if (colorseg_flags[j] && (seg & 1))
                    {
                        rgb = *(uint32_t *)&color_table[j];
                        break;
                    }
                    seg >>= 1;
                }
                *out++ = rgb;
            }
            colorseg_tex.update(colorseg_image);
            full_rect();
        }
    }

    // Solid white texture
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);

    // Spans and groups
    if (show_groups || show_span_detail)
    {
        for (int i = 0; i < Vision::Num_Colors; ++i)
        {
            QMutexLocker ml(&_vision->spanner[i]->mutex);

            if (group_flags[i])
            {
                draw_spanner(_vision->spanner[i]);
            }
        }
    }

	if (_setup)
	{
		_setup->draw();
	}

    // Statistics
    QTime now = QTime::currentTime();
    unsigned int delta_ms = _last_frame_time.msecsTo(now);
    _last_frame_time = now;

    unsigned int frame_count = camera_thread->frame_count();
    unsigned int skipped = frame_count - last_frame_count;
    last_frame_count = frame_count;

    if (debug)
    {
        unsigned int proc_ms = camera_thread->frame_ms();
        printf("\rproc %4dms %6.2f fps  view %4d ms %6.2f fps  skip %d       ",
            proc_ms, 1000.0 / proc_ms,
            delta_ms, 1000.0 / delta_ms,
            skipped);
        fflush(stdout);
    }
}

void GL_Camera_View::keyPressEvent(QKeyEvent *e)
{
    switch (e->key())
    {
        case Qt::Key_Escape:
            // Select no color to edit
            _win->edit_nothing();
            break;

        case ' ':
            // Toggle paused
            camera_thread->pause(!camera_thread->paused());
            break;

        case '1':
            // Default scale/offset
            scale = min((float)width() / _frame_width, (float)height() / _frame_height);
            offset_x = 0;
            offset_y = 0;
            updateGL();
            break;
    }
}

void GL_Camera_View::mousePressEvent(QMouseEvent *e)
{
	Geometry::Point2d pos = frame_position(e->pos());

	if (_setup)
	{
		_setup->mousePressEvent(e, pos);
	} else {
        mouse_edit_color(pos, e->buttons());
	}

    _last_mouse_pos = pos;
}

void GL_Camera_View::mouseDoubleClickEvent(QMouseEvent *e)
{
	Geometry::Point2d pos = frame_position(e->pos());

	if (_setup)
	{
		_setup->mouseDoubleClickEvent(e, pos);
	}

	_last_mouse_pos = pos;
}

void GL_Camera_View::mouseMoveEvent(QMouseEvent *e)
{
	Geometry::Point2d pos = frame_position(e->pos());
    if (e->buttons() & Qt::MidButton)
    {
        // Pan
        offset_x += _last_mouse_pos.x - pos.x;
        offset_y += _last_mouse_pos.y - pos.y;

        updateGL();
    } else {
        _last_mouse_pos = pos;
    }

    if (_setup)
    {
    	_setup->mouseMoveEvent(e, pos);
    } else {
        mouse_edit_color(pos, e->buttons());
    }

    mouse_moved((int)pos.x, (int)pos.y);
}

void GL_Camera_View::wheelEvent(QWheelEvent *e)
{
	Geometry::Point2d pos = frame_position(e->pos());
    scale *= pow(2, (float)e->delta() / (120 * 2));
#if 0
    if (scale < 1)
    {
        scale = 1;
    } else if (scale > 8)
    {
        scale = 8;
    }
#endif

    // Change offset to keep the mouse in the same position
    Geometry::Point2d delta = frame_position(e->pos()) - pos;
    offset_x -= delta.x;
    offset_y -= delta.y;

    _last_mouse_pos = pos;

    updateGL();
}

void GL_Camera_View::mouse_edit_color(Geometry::Point2d &pos, Qt::MouseButtons buttons)
{
    if (edit_color >= 0)
    {
        QRgb in_color = camera_thread->get_pixel((int)pos.x, (int)pos.y);
        uint8_t &lut = _vision->colorseg->lut(in_color);
        uint8_t color_bit = 1 << edit_color;

        if (buttons == Qt::LeftButton)
        {
            lut |= color_bit;
            colorseg_changed(in_color);
        } else if (buttons == Qt::RightButton)
        {
            lut &= ~color_bit;
            colorseg_changed(in_color);
        }
    }
}
