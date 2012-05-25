#include "Field.hpp"

#include <physics/PhysicsConstants.hpp>
#include <vector>
#include <math.h>

using namespace std;

static const float PI = 3.14159265;

void Field::renderField(){
	//glEnable (GL_BLEND);
	//glBlendFunc (GL_SRC_ALPHA, GL_SRC_COLOR);

	//Field green
	glColor4f(0.,1.0,0.,1.0);
	glBegin(GL_POLYGON);
		glVertex3f(_x+Sim_Floor_Width/2.f, 0.01, _y+Sim_Floor_Length/2.f);
		glVertex3f(_x-Sim_Floor_Width/2.f, 0.01, _y+Sim_Floor_Length/2.f);
		glVertex3f(_x-Sim_Floor_Width/2.f, 0.01, _y-Sim_Floor_Length/2.f);
		glVertex3f(_x+Sim_Floor_Width/2.f, 0.01, _y-Sim_Floor_Length/2.f);
	glEnd();

	float lineHeight = 0.02;
	//Center circle
	glColor4f (1.0, 1.0, 1.0, 1.0);
	renderArc(_x,_y,0,PI*2,lineHeight,Sim_Field_CenterRadius,Sim_Field_LineWidth,360);

	//Field Boundary Lines
	renderVerticalLine(_x+Sim_Field_Width/2.f, _y+Sim_Field_Length/2.f, _x+Sim_Field_Width/2.f, _y-Sim_Field_Length/2.f, lineHeight, Sim_Field_LineWidth);
	renderVerticalLine(_x-Sim_Field_Width/2.f, _y+Sim_Field_Length/2.f, _x-Sim_Field_Width/2.f, _y-Sim_Field_Length/2.f, lineHeight, Sim_Field_LineWidth);
	renderHorizontalLine(_x+Sim_Field_Width/2.f, _y+Sim_Field_Length/2.f, _x-Sim_Field_Width/2.f, _y+Sim_Field_Length/2.f, lineHeight, Sim_Field_LineWidth);
	renderHorizontalLine(_x+Sim_Field_Width/2.f, _y-Sim_Field_Length/2.f, _x-Sim_Field_Width/2.f, _y-Sim_Field_Length/2.f, lineHeight, Sim_Field_LineWidth);

	//Center Line
	renderHorizontalLine(_x+Sim_Field_Width/2.f,_y,_x-Sim_Field_Width/2.f,_y,lineHeight,Sim_Field_LineWidth);

	//Goal Arc //0 radians is in the Z direction (forward)
	renderArc(_x-(Sim_Field_GoalFlat/2.f),_y-Sim_Field_Length/2.f,PI*3/2.f,PI*2.f,lineHeight,Sim_Field_ArcRadius,Sim_Field_LineWidth*2,90);
	renderArc(_x+(Sim_Field_GoalFlat/2.f),_y-Sim_Field_Length/2.f,0,PI/2.f,lineHeight,Sim_Field_ArcRadius,Sim_Field_LineWidth*2,90);
	renderHorizontalLine(_x-Sim_Field_GoalFlat/2.f,_y-Sim_Field_Length/2.f+Sim_Field_ArcRadius,_x+Sim_Field_GoalFlat/2.f,_y-Sim_Field_Length/2.f+Sim_Field_ArcRadius,lineHeight, Sim_Field_LineWidth);

	renderArc(_x-(Sim_Field_GoalFlat/2.f),_y+Sim_Field_Length/2.f,PI,PI*3/2.f,lineHeight,Sim_Field_ArcRadius,Sim_Field_LineWidth*2,90);
	renderArc(_x+(Sim_Field_GoalFlat/2.f),_y+Sim_Field_Length/2.f,PI/2.f,PI,lineHeight,Sim_Field_ArcRadius,Sim_Field_LineWidth*2,90);
	renderHorizontalLine(_x-Sim_Field_GoalFlat/2.f,_y+Sim_Field_Length/2.f-Sim_Field_ArcRadius,_x+Sim_Field_GoalFlat/2.f,_y+Sim_Field_Length/2.f-Sim_Field_ArcRadius,lineHeight, Sim_Field_LineWidth);

	//Goal Area
	glColor4f(1.0,1.0,0,1.0); //Yellow
	renderVerticalLine(_x-Sim_Field_GoalWidth/2.f,_y-Sim_Field_Length/2.f,_x-Sim_Field_GoalWidth/2.f,_y-(Sim_Field_Length/2.f+Sim_Field_GoalDepth),lineHeight,Sim_Field_LineWidth);
	renderVerticalLine(_x+Sim_Field_GoalWidth/2.f,_y-Sim_Field_Length/2.f,_x+Sim_Field_GoalWidth/2.f,_y-(Sim_Field_Length/2.f+Sim_Field_GoalDepth),lineHeight,Sim_Field_LineWidth);
	renderHorizontalLine(_x-Sim_Field_GoalWidth/2.f,_y-(Sim_Field_Length/2.f+Sim_Field_GoalDepth),_x+Sim_Field_GoalWidth/2.f,_y-(Sim_Field_Length/2.f+Sim_Field_GoalDepth),lineHeight,Sim_Field_LineWidth);

	glColor4f(0,0,1.0,1.0); //Blue
	renderVerticalLine(_x-Sim_Field_GoalWidth/2.f,_y+Sim_Field_Length/2.f,_x-Sim_Field_GoalWidth/2.f,_y+(Sim_Field_Length/2.f+Sim_Field_GoalDepth),lineHeight,Sim_Field_LineWidth);
	renderVerticalLine(_x+Sim_Field_GoalWidth/2.f,_y+Sim_Field_Length/2.f,_x+Sim_Field_GoalWidth/2.f,_y+(Sim_Field_Length/2.f+Sim_Field_GoalDepth),lineHeight,Sim_Field_LineWidth);
	renderHorizontalLine(_x-Sim_Field_GoalWidth/2.f,_y+(Sim_Field_Length/2.f+Sim_Field_GoalDepth),_x+Sim_Field_GoalWidth/2.f,_y+(Sim_Field_Length/2.f+Sim_Field_GoalDepth),lineHeight,Sim_Field_LineWidth);

	//glDisable(GL_BLEND);
}

void Field::renderVerticalLine(float x1, float z1, float x2, float z2, float height, float lineWidth){
	glBegin(GL_POLYGON);
		glVertex3f(x1-lineWidth,height,z1);
		glVertex3f(x1+lineWidth,height,z1);
		glVertex3f(x2-lineWidth,height,z2);
		glVertex3f(x2+lineWidth,height,z2);
	glEnd();
}

void Field::renderHorizontalLine(float x1, float z1, float x2, float z2, float height, float lineWidth){
	glBegin(GL_POLYGON);
		glVertex3f(x1,height,z1-lineWidth);
		glVertex3f(x1,height,z1+lineWidth);
		glVertex3f(x2,height,z2-lineWidth);
		glVertex3f(x2,height,z2+lineWidth);
	glEnd();
}

void Field::renderArc(float x, float z, float angle1, float angle2, float height, float radius, float lineWidth, int numPoints){
	glBegin(GL_TRIANGLE_STRIP);
	float delta = (angle2-angle1)/(float)numPoints;
	for(float theta = angle1; theta <= angle2; theta += delta){
		glVertex3f(x+sin(theta)*(radius-lineWidth), height, z+cos(theta)*(radius-lineWidth/2.f));
		glVertex3f(x+sin(theta)*radius, height, z+cos(theta)*(radius+lineWidth/2.f));
	}
	glEnd();
}

Field::Field(Environment* env) :
	Entity(env),
	_x(0),
	_y(0)
{
}

Field::~Field()
{
	
}
