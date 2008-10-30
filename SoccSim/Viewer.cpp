#include "Viewer.hpp"

Viewer::Viewer(Env* env, QWidget* parent) :
	QGLWidget(parent), _env(env)
{
	this->setFixedSize(800, 600);
	
	connect(&_repaint, SIGNAL(timeout()), this, SLOT(updateGL()));
	_repaint.start(30);
}

Viewer::~Viewer()
{

}

void Viewer::initializeGL()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LINE_SMOOTH);
}

void Viewer::resizeGL(int w, int h)
{
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	//glOrtho(-5, 5, -3, 3, -1, 1);
	
	gluPerspective(60.0, w / h, 1, 1000);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//eyeXYZ, targetXYZ, upXYZ
	//gluLookAt(-1, -1, 1, 0, 0, 0, 0, 0, 1);
	gluLookAt(-5, -5, 5, 0, 0, 0, 0, 0, 1);
	//gluLookAt(0, 0, 2, 0, 0, 0, 0, 0, 1);
	//gluLookAt(.7, .7, .7, 0, 0, 0, 0, 0, 1);
}

void Viewer::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	renderData(_env->dbgRenderable());
}

void Viewer::renderData(const NxDebugRenderable& data) const
{
	glLineWidth(1.0f);
    // Render points
	{
		NxU32 NbPoints = data.getNbPoints();
		const NxDebugPoint* Points = data.getPoints();
		glBegin(GL_POINTS);
		while(NbPoints--)
		{
			setupColor(Points->color);
			glVertex3fv(&Points->p.x);
			Points++;
		}
		glEnd();
	}

	// Render lines
	{
		NxU32 NbLines = data.getNbLines();
		const NxDebugLine* Lines = data.getLines();
		glBegin(GL_LINES);
		while(NbLines--)
		{
			setupColor(Lines->color);
			glVertex3fv(&Lines->p0.x);
			glVertex3fv(&Lines->p1.x);
			Lines++;
		}
		glEnd();
	}

	// Render triangles
	{
		NxU32 NbTris = data.getNbTriangles();
		const NxDebugTriangle* Triangles = data.getTriangles();
		glBegin(GL_TRIANGLES);
		while(NbTris--)
		{
			setupColor(Triangles->color);
			glVertex3fv(&Triangles->p0.x);
			glVertex3fv(&Triangles->p1.x);
			glVertex3fv(&Triangles->p2.x);
			Triangles++;
		}
		glEnd();
	}
}

void Viewer::setupColor(NxU32 color) const
{
	NxF32 Blue = NxF32((color)&0xff)/255.0f;
	NxF32 Green = NxF32((color>>8)&0xff)/255.0f;
	NxF32 Red = NxF32((color>>16)&0xff)/255.0f;

	glColor3f(Red, Green, Blue);
}
