#include <QtGui>
#include <QtOpenGL>

#include <math.h>

#include "SimRenderView.hpp"
#include "VizObject.hpp"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

using namespace rendering;

SimRenderView::SimRenderView(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    _xRot = 0;
    _yRot = 0;
    _zRot = 0;

    _scale = 0.13;

    _backgroundColor = QColor::fromCmykF(0.39, 0.39, 0.0, 0.0);
}

SimRenderView::~SimRenderView()
{
	qDeleteAll(_entities);
}

QSize SimRenderView::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize SimRenderView::sizeHint() const
{
    return QSize(800, 600);
}

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

void SimRenderView::setXRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != _xRot) {
        _xRot = angle;
        emit xRotationChanged(angle);
        updateGL();
    }
}

void SimRenderView::setYRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != _yRot) {
        _yRot = angle;
        emit yRotationChanged(angle);
        updateGL();
    }
}

void SimRenderView::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != _zRot) {
        _zRot = angle;
        emit zRotationChanged(angle);
        updateGL();
    }
}

void SimRenderView::initializeGL()
{
    qglClearColor(_backgroundColor.dark());

    // Add objects

    // Add a RoboCup Field
    VizObject* field = new RCField(this, _scale);
    _entities.push_back(field);

    // Add a RoboCup Robot for each side
    VizObject* robotBlue = new RobotBody(this, _scale);
    robotBlue->translate(QVector3D(1.0, 0.0, 0.0));
    robotBlue->setColor(Qt::blue);
    _entities.push_back(robotBlue);

//    VizObject* robotYellow = new RobotBody(this);
//    robotYellow->setColor(Qt::yellow);
//    _entities.push_back(robotYellow);

    // Add a Ball
//    VizObject* ball = new Ball(this);
//    _entities.push_back(ball);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_MULTISAMPLE);
    static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

void SimRenderView::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // Default camera is top-down

    // Setup camera
    glTranslatef(0.0, 0.0, -10.0);

    // axis-angle rotations
    glRotatef(_xRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(_yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(_zRot / 16.0, 0.0, 0.0, 1.0);

    // render all of the objects
    for (int i=0; i<_entities.size(); ++i)
    	_entities[i]->draw();
}

void SimRenderView::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
    glMatrixMode(GL_MODELVIEW);
}

void SimRenderView::mousePressEvent(QMouseEvent *event)
{
    _lastPos = event->pos();
}

void SimRenderView::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - _lastPos.x();
    int dy = event->y() - _lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(_xRot + 8 * dy);
        setYRotation(_yRot + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(_xRot + 8 * dy);
        setZRotation(_zRot + 8 * dx);
    }
    _lastPos = event->pos();
}
