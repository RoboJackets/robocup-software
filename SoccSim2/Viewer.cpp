#include "Viewer.hpp"

Viewer::Viewer(QWidget* parent) :
	QGLWidget(parent)
{
	this->setFixedSize(800, 600);
	
	connect(&_repaint, SIGNAL(timeout()), this, SLOT(updateGL()));
	_repaint.start(30);

	env.start();
}

Viewer::~Viewer()
{

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
	//gluLookAt(-1, -1, 1, 0, 0, 0, 0, 0, 1);
	gluLookAt(.7, .7, .7, 0, 0, 0, 0, 0, 1);
	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LINE_SMOOTH);
	
	//glEnable(GL_POLYGON_SMOOTH);
	//glEnable(GL_BLEND);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
#if 0
    float light0_pos[] = {0, 0, 3, 1};

    const float am = .1;
    float ambient[] = {am, am, am, 1};

    glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glEnable(GL_LIGHT0);
    
    glEnable(GL_LIGHTING);
#endif

}

void Viewer::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	env.step();
	
	//force the entities to redraw themselves
	//env.redraw();
	
	renderData(env.dbgRenderable());
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
