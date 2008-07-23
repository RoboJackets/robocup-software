#include "Field.hpp"

using namespace Physics;

Field::Field(dWorldID world, dSpaceID space)
	: Entity(world, space, false) //field does not move
{
	//destroy old geometry
	//dGeomDestroy(_geomID);

	//create a plane for the field/floor
	//a,b,c, d
	_geomID = dCreatePlane(space, 0, 0, 1, 0);
	dGeomSetData(_geomID, (void*)Entity::Floor);

	//wall thickness
	const float thickness = .01;

	//boundary
	wall(space, thickness, FLOOR_WIDTH, -FLOOR_LENGTH/2.0, 0);
	wall(space, thickness, FLOOR_WIDTH, FLOOR_LENGTH/2.0, 0);
	wall(space, FLOOR_LENGTH, thickness, 0, FLOOR_WIDTH/2.0);
	wall(space, FLOOR_LENGTH, thickness, 0, -FLOOR_WIDTH/2.0);

	/// create goals ///
	//yellow
	wall(space, GOAL_THICKNESS, GOAL_WIDTH, -FIELD_LENGTH/2.0-GOAL_DEPTH, 0);
	wall(space, GOAL_DEPTH, GOAL_THICKNESS, -FIELD_LENGTH/2.0 - GOAL_DEPTH/2.0f, GOAL_WIDTH/2.0);
	wall(space, GOAL_DEPTH, GOAL_THICKNESS, -FIELD_LENGTH/2.0 - GOAL_DEPTH/2.0f, -GOAL_WIDTH/2.0);

	//blue
	wall(space, GOAL_THICKNESS, GOAL_WIDTH, FIELD_LENGTH/2.0 + GOAL_DEPTH, 0);
	wall(space, GOAL_DEPTH, GOAL_THICKNESS, FIELD_LENGTH/2.0 + GOAL_DEPTH/2.0f, GOAL_WIDTH/2.0);
	wall(space, GOAL_DEPTH, GOAL_THICKNESS, FIELD_LENGTH/2.0 + GOAL_DEPTH/2.0f, -GOAL_WIDTH/2.0);
}

Field::~Field()
{

}

void Field::wall(dSpaceID space, float w, float l, float x, float y)
{
	static const float height = .2;

	dGeomID wl = dCreateBox(space, w, l, height);
	dGeomSetPosition(wl, x, y, 0);
	dGeomSetData(wl, (void*)Entity::Wall);
}

void Field::paint()
{
	//field is assumed @ 0,0,0
	dReal pos[] = {0,0,0,0,0,0};

	glPushMatrix();
		glTranslatef(pos[0]-FLOOR_LENGTH/2.0f, pos[1]-FLOOR_WIDTH/2.0f, pos[2]);

		//floor
		glColor3ub(0, 150, 0);
		glBegin(GL_QUADS);
			glVertex2f(0, 0);
			glVertex2f(0, FLOOR_WIDTH);
			glVertex2f(FLOOR_LENGTH, FLOOR_WIDTH);
			glVertex2f(FLOOR_LENGTH, 0);
		glEnd();

		glTranslatef(FIELD_DEADSPACE, FIELD_DEADSPACE, 0);

		glColor3ub(255, 255, 255);
		glLineWidth(2.0);

		//outer boundary
		glBegin(GL_LINE_LOOP);
			glVertex2f(0,0);
			glVertex2f(0, FIELD_WIDTH);
			glVertex2f(FIELD_LENGTH, FIELD_WIDTH);
			glVertex2f(FIELD_LENGTH, 0);
		glEnd();

		glTranslatef(FIELD_LENGTH/2.0, FIELD_WIDTH/2.0, 0);

		//centerline
		glBegin(GL_LINES);
			glVertex2f(0, -FIELD_WIDTH/2.0);
			glVertex2f(0, FIELD_WIDTH/2.0);
		glEnd();

		const float step = 2*M_PI/15; //15 sides
		//center circle
		glBegin(GL_LINE_LOOP);
			for (float i=0 ; i<2*M_PI ; i+=step)
			{
				glVertex2f(ARC_RADIUS * cos(i), ARC_RADIUS * sin(i));
			}
		glEnd();

		//blue goal
		glColor3ub(0, 0, 255);
		glTranslatef(FIELD_LENGTH/2.0f+GOAL_DEPTH, 0, 0);
		glBegin(GL_QUADS);
			glVertex3f(0, -GOAL_WIDTH/2.0f,  0);
			glVertex3f(0, -GOAL_WIDTH/2.0f, .2f);
			glVertex3f(0,  GOAL_WIDTH/2.0f, .2f);
			glVertex3f(0,  GOAL_WIDTH/2.0f,  0);

			glVertex3f(-GOAL_DEPTH, -GOAL_WIDTH/2.0f, 0);
			glVertex3f(0, -GOAL_WIDTH/2.0f, 0);
			glVertex3f(0, -GOAL_WIDTH/2.0f, .2f);
			glVertex3f(-GOAL_DEPTH, -GOAL_WIDTH/2.0f, .2f);

			glVertex3f(-GOAL_DEPTH, GOAL_WIDTH/2.0f, 0);
			glVertex3f(0, GOAL_WIDTH/2.0f, 0);
			glVertex3f(0, GOAL_WIDTH/2.0f, .2f);
			glVertex3f(-GOAL_DEPTH, GOAL_WIDTH/2.0f, .2f);
		glEnd();

		//yellow goal
		glColor3ub(255, 255, 0);
		glTranslatef(-FIELD_LENGTH-2*GOAL_DEPTH, 0, 0);
		glBegin(GL_QUADS);
			glVertex3f(0, -GOAL_WIDTH/2.0f,  0);
			glVertex3f(0, -GOAL_WIDTH/2.0f, .2f);
			glVertex3f(0,  GOAL_WIDTH/2.0f, .2f);
			glVertex3f(0,  GOAL_WIDTH/2.0f,  0);

			glVertex3f(GOAL_DEPTH, -GOAL_WIDTH/2.0f, 0);
			glVertex3f(0, -GOAL_WIDTH/2.0f, 0);
			glVertex3f(0, -GOAL_WIDTH/2.0f, .2f);
			glVertex3f(GOAL_DEPTH, -GOAL_WIDTH/2.0f, .2f);

			glVertex3f(GOAL_DEPTH, GOAL_WIDTH/2.0f, 0);
			glVertex3f(0, GOAL_WIDTH/2.0f, 0);
			glVertex3f(0, GOAL_WIDTH/2.0f, .2f);
			glVertex3f(GOAL_DEPTH, GOAL_WIDTH/2.0f, .2f);
		glEnd();

	glPopMatrix();
}
