#pragma once

#include <QGLWidget>

namespace rendering {
class VizObject;
} // \namespace rendering

class SimRenderView : public QGLWidget
{
	Q_OBJECT

public:
	SimRenderView(QWidget *parent = 0);
	~SimRenderView();

	QSize minimumSizeHint() const;
	QSize sizeHint() const;

public slots:
	void setXRotation(int angle);
	void setYRotation(int angle);
	void setZRotation(int angle);

signals:
	void xRotationChanged(int angle);
	void yRotationChanged(int angle);
	void zRotationChanged(int angle);

protected:
	void initializeGL();
	void paintGL();
	void resizeGL(int width, int height);
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);

private:

	QVector<rendering::VizObject*> _entities;

	int xRot;
	int yRot;
	int zRot;
	QPoint lastPos;
	QColor qtGreen;
	QColor qtPurple;
};
