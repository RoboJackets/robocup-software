#include "Viewer.hpp"

#include <Sizes.h>

Viewer::Viewer(QWidget* parent) :
	QGLWidget(parent)
{
	this->setFixedSize(800, 600);

	_env = 0;

	connect(&_repaint, SIGNAL(timeout()), this, SLOT(updateGL()));
	_repaint.start(30);
}

Viewer::~Viewer()
{

}

void Viewer::setEnv(Physics::Env* env)
{
	_env = env;
}

void Viewer::initializeGL()
{
#if 0
	float light0_pos[] = {0, 0, 3, 1};

	const float am = .6;
	float ambient[] = {am, am, am, 1};

	glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);

	glEnable(GL_LIGHT0);

	glEnable(GL_LIGHTING);
#endif

	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_COLOR_MATERIAL);

	//glEnable(GL_VERTEX_ARRAY);
	//glEnable(GL_NORMAL_ARRAY);

	//glShadeModel(GL_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
	//glEnable(GL_POLYGON_SMOOTH);

	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void Viewer::resizeGL(int w, int h)
{
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, w / h, 1, 1000);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//eyeXYZ, targetXYZ, upXYZ
	gluLookAt(-4, -4, 4, 0, 0, 0, 0, 0, 1);
	//gluLookAt(-1, -1, 1, 0, 0, 0, 0, 0, 1);
	//gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);
}

void Viewer::paintGL()
{
	if (isVisible())
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
		if (_env)
		{
			QVector<Physics::Entity*> entities = _env->entities();
			Q_FOREACH(Physics::Entity* e, entities)
			{
				e->paint();
			}
		}
	}
}
