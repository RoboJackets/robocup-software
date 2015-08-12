#pragma once

#include <QGLWidget>
#include <GL/glu.h>
#include <Eigen/Geometry>

class QuaternionDemo: public QGLWidget
{
public:
	QuaternionDemo(QWidget *parent = nullptr);

	bool initialized;
	Eigen::Quaternionf ref;
	Eigen::Quaternionf q;

protected:
	void paintGL() override;
};

QuaternionDemo::QuaternionDemo(QWidget* parent)
{
	initialized = false;
	ref = Eigen::Quaternionf::Identity();
}

void QuaternionDemo::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (double)width() / height(), 0.1, 10);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0, 0, -5);

	glRotatef(-90, 1, 0, 0);
	glRotatef(90, 0, 0, 1);

	Eigen::Transform<float, 3, Eigen::Affine> t_ref(ref.conjugate());
	glMultMatrixf(t_ref.data());
	Eigen::Transform<float, 3, Eigen::Affine> t_q(q);
	glMultMatrixf(t_q.data());

	glEnable(GL_DEPTH_TEST);
	glBegin(GL_QUADS);
		glColor3f(1, 1, 1);
		glVertex3f(-1, -1, -1);
		glVertex3f(1, -1, -1);
		glVertex3f(1, 1, -1);
		glVertex3f(-1, 1, -1);

		glColor3f(1, 0, 0);
		glVertex3f(-1, -1, 1);
		glVertex3f(1, -1, 1);
		glVertex3f(1, 1, 1);
		glVertex3f(-1, 1, 1);

		glColor3f(0, 1, 0);
		glVertex3f(-1, -1, 1);
		glVertex3f(-1, -1, -1);
		glVertex3f(-1, 1, -1);
		glVertex3f(-1, 1, 1);

		glColor3f(1, 0.5f, 0);
		glVertex3f(1, -1, 1);
		glVertex3f(1, -1, -1);
		glVertex3f(1, 1, -1);
		glVertex3f(1, 1, 1);

		glColor3f(0, 0, 1);
		glVertex3f(-1, -1, 1);
		glVertex3f(-1, -1, -1);
		glVertex3f(1, -1, -1);
		glVertex3f(1, -1, 1);

		glColor3f(1, 0, 0.5f);
		glVertex3f(-1, 1, 1);
		glVertex3f(-1, 1, -1);
		glVertex3f(1, 1, -1);
		glVertex3f(1, 1, 1);
	glEnd();
}
