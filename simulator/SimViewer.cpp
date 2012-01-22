#include <QtGui>
#include <QtOpenGL>

#include "SimViewer.hpp"

SimViewer::SimViewer(QWidget *parent)
: QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
}

SimViewer::~SimViewer()
{
}

QSize SimViewer::minimumSizeHint() const
{
	return QSize(640, 480);
}

QSize SimViewer::sizeHint() const
{
	return QSize(640, 480);
}

void SimViewer::initializeGL()
{
}

void SimViewer::paintGL()
{
}

void SimViewer::resizeGL(int width, int height)
{
}
