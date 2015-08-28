#include "Field.hpp"
#include <physics/Environment.hpp>
#include <vector>
#include <math.h>

using namespace std;

Field::Field(Environment* env)
    : Entity(env),
      _indexVertexArrays(nullptr),
      _vertices(nullptr),
      _simEngine(env->getSimEngine()),
      _x(0),
      _y(0) {}

Field::~Field() {
    delete _indexVertexArrays;
    delete[] _vertices;
}

void Field::initPhysics() {
    btCollisionShape* groundShape;  // = new btBoxShape(btVector3(50, 3, 50));
    //_simEngine->addCollisionShape(groundShape);
    btTransform tr;
    tr.setIdentity();

    // use triangle mesh for ground
    int i;

    const float TRIANGLE_SIZE = 20.f;

    // create a triangle-mesh ground
    int vertStride = sizeof(btVector3);
    int indexStride = 3 * sizeof(int);

    const int NUM_VERTS_X = 20;
    const int NUM_VERTS_Y = 20;
    const int totalVerts = NUM_VERTS_X * NUM_VERTS_Y;

    const int totalTriangles = 2 * (NUM_VERTS_X - 1) * (NUM_VERTS_Y - 1);

    _vertices = new btVector3[totalVerts];
    int* gIndices = new int[totalTriangles * 3];

    for (i = 0; i < NUM_VERTS_X; i++) {
        for (int j = 0; j < NUM_VERTS_Y; j++) {
            float height = 0.f;
            _vertices[i + j * NUM_VERTS_X].setValue(
                (i - NUM_VERTS_X * 0.5f) * TRIANGLE_SIZE, height,
                (j - NUM_VERTS_Y * 0.5f) * TRIANGLE_SIZE);
        }
    }

    int index = 0;
    for (i = 0; i < NUM_VERTS_X - 1; i++) {
        for (int j = 0; j < NUM_VERTS_Y - 1; j++) {
            gIndices[index++] = j * NUM_VERTS_X + i;
            gIndices[index++] = j * NUM_VERTS_X + i + 1;
            gIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;

            gIndices[index++] = j * NUM_VERTS_X + i;
            gIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;
            gIndices[index++] = (j + 1) * NUM_VERTS_X + i;
        }
    }

    _indexVertexArrays = new btTriangleIndexVertexArray(
        totalTriangles, gIndices, indexStride, totalVerts,
        (btScalar*)&_vertices[0].x(), vertStride);

    bool useQuantizedAabbCompression = true;
    groundShape = new btBvhTriangleMeshShape(_indexVertexArrays,
                                             useQuantizedAabbCompression);

    tr.setOrigin(btVector3(0, 0.f, 0));

    _simEngine->addCollisionShape(groundShape);

    // color
    btVector3* color = new btVector3(0.5f, 0.5f, 0.5f);
    groundShape->setUserPointer(color);

    // create ground object
    btRigidBody* ground = _simEngine->localCreateRigidBody(0, tr, groundShape);
    ground->setFriction(10.0);
    ground->setRestitution(0);

    // create walls
    const float halfWidth = 0.1 * scaling;
    const float halfHeight = 0.05 * scaling;

    btCollisionShape* longWallShape = new btBoxShape(
        btVector3(halfWidth, halfHeight,
                  Field_Dimensions::Current_Dimensions.FloorLength() / 2.f +
                      2 * halfWidth));
    btCollisionShape* wideWallShape = new btBoxShape(
        btVector3(Field_Dimensions::Current_Dimensions.FloorWidth() / 2.f,
                  halfHeight, halfWidth));
    longWallShape->setMargin(0.004 * scaling);
    wideWallShape->setMargin(0.004 * scaling);

    _simEngine->addCollisionShape(longWallShape);
    _simEngine->addCollisionShape(wideWallShape);

    tr.setOrigin(btVector3(
        Field_Dimensions::Current_Dimensions.FloorWidth() / 2.f + halfWidth,
        halfHeight, 0));
    _wallBodies[0] = _simEngine->localCreateRigidBody(0, tr, longWallShape);

    tr.setOrigin(btVector3(
        -Field_Dimensions::Current_Dimensions.FloorWidth() / 2.f - halfWidth,
        halfHeight, 0));
    _wallBodies[1] = _simEngine->localCreateRigidBody(0, tr, longWallShape);

    tr.setOrigin(btVector3(
        0, halfHeight,
        Field_Dimensions::Current_Dimensions.FloorLength() / 2.f + halfWidth));
    _wallBodies[2] = _simEngine->localCreateRigidBody(0, tr, wideWallShape);

    tr.setOrigin(btVector3(
        0, halfHeight,
        -Field_Dimensions::Current_Dimensions.FloorLength() / 2.f - halfWidth));
    _wallBodies[3] = _simEngine->localCreateRigidBody(0, tr, wideWallShape);

    // color
    color = new btVector3(1, 1, 1);
    longWallShape->setUserPointer(color);
    wideWallShape->setUserPointer(color);

    // create goal walls for blue
    btBoxShape* goalBackShape = new btBoxShape(
        btVector3(Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f +
                      Sim_GoalWall_Width,
                  Sim_GoalWall_Height / 2.f, Sim_GoalWall_Width / 2.f));
    btBoxShape* goalSideShape = new btBoxShape(
        btVector3(Sim_GoalWall_Width / 2.f, Sim_GoalWall_Height / 2.f,
                  Field_Dimensions::Current_Dimensions.GoalDepth() / 2.f));
    goalBackShape->setMargin(0.004 * scaling);
    goalSideShape->setMargin(0.004 * scaling);

    _simEngine->addCollisionShape(goalBackShape);
    _simEngine->addCollisionShape(goalSideShape);

    btVector3 backPos =
        btVector3(0, Sim_GoalWall_Height / 2.f,
                  Field_Dimensions::Current_Dimensions.Length() / 2.f +
                      Sim_GoalWall_Width / 2.f +
                      Field_Dimensions::Current_Dimensions.GoalDepth());
    btVector3 sidePos =
        btVector3(Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f +
                      Sim_GoalWall_Width / 2.f,
                  Sim_GoalWall_Height / 2.f,
                  Field_Dimensions::Current_Dimensions.Length() / 2.f +
                      Field_Dimensions::Current_Dimensions.GoalDepth() / 2.f);

    tr.setOrigin(backPos);
    _blueGoalBodies[0] = _simEngine->localCreateRigidBody(0, tr, goalBackShape);

    tr.setOrigin(sidePos);
    _blueGoalBodies[1] = _simEngine->localCreateRigidBody(0, tr, goalSideShape);

    sidePos.setX(-sidePos.x());
    tr.setOrigin(sidePos);
    _blueGoalBodies[2] = _simEngine->localCreateRigidBody(0, tr, goalSideShape);

    // color blue
    color = new btVector3(0, 0, 1);
    goalBackShape->setUserPointer(color);
    goalSideShape->setUserPointer(color);

    // create goal walls for yellow
    /// FIXME: Object colors are stored in the userPointer of btCollisionShapes.
    /// The side effect is that new collision shapes need to be created each
    /// time.
    /// Alternatives are to store them in btCollisionObject's userObjectPointer
    /// or
    /// plain figure out a better way to do this.
    goalBackShape = new btBoxShape(*goalBackShape);
    goalSideShape = new btBoxShape(*goalSideShape);
    goalBackShape->setMargin(0.004 * scaling);
    goalSideShape->setMargin(0.004 * scaling);

    backPos.setZ(-backPos.z());
    sidePos.setZ(-sidePos.z());

    tr.setOrigin(backPos);
    _yellowGoalBodies[0] =
        _simEngine->localCreateRigidBody(0, tr, goalBackShape);

    tr.setOrigin(sidePos);
    _yellowGoalBodies[1] =
        _simEngine->localCreateRigidBody(0, tr, goalSideShape);

    sidePos.setX(-sidePos.x());
    tr.setOrigin(sidePos);
    _yellowGoalBodies[2] =
        _simEngine->localCreateRigidBody(0, tr, goalSideShape);

    // color blue
    color = new btVector3(1, 1, 0.5f);
    goalBackShape->setUserPointer(color);
    goalSideShape->setUserPointer(color);
}

void Field::renderField() {
    int debug_mode =
        _simEngine->dynamicsWorld()->getDebugDrawer()->getDebugMode();
    if (debug_mode & btIDebugDraw::DBG_DrawWireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_TEXTURE_2D);
    }

    // Field green
    float fieldHeight = 0.001 * scaling;  // Strictly for rendering purposes
    glColor4f(0., 1.0, 0., 1.0);
    glBegin(GL_POLYGON);
    glVertex3f(_x + Field_Dimensions::Current_Dimensions.FloorWidth() / 2.f,
               fieldHeight,
               _y + Field_Dimensions::Current_Dimensions.FloorLength() / 2.f);
    glVertex3f(_x - Field_Dimensions::Current_Dimensions.FloorWidth() / 2.f,
               fieldHeight,
               _y + Field_Dimensions::Current_Dimensions.FloorLength() / 2.f);
    glVertex3f(_x - Field_Dimensions::Current_Dimensions.FloorWidth() / 2.f,
               fieldHeight,
               _y - Field_Dimensions::Current_Dimensions.FloorLength() / 2.f);
    glVertex3f(_x + Field_Dimensions::Current_Dimensions.FloorWidth() / 2.f,
               fieldHeight,
               _y - Field_Dimensions::Current_Dimensions.FloorLength() / 2.f);
    glEnd();

    float lineHeight = 0.002 * scaling;
    // Center circle
    glColor4f(1.0, 1.0, 1.0, 1.0);
    renderArc(_x, _y, 0, M_PI * 2, lineHeight,
              Field_Dimensions::Current_Dimensions.CenterRadius(),
              Field_Dimensions::Current_Dimensions.LineWidth(), 360);

    // Field Boundary Lines
    renderVerticalLine(_x + Field_Dimensions::Current_Dimensions.Width() / 2.f,
                       _y + Field_Dimensions::Current_Dimensions.Length() / 2.f,
                       _x + Field_Dimensions::Current_Dimensions.Width() / 2.f,
                       _y - Field_Dimensions::Current_Dimensions.Length() / 2.f,
                       lineHeight,
                       Field_Dimensions::Current_Dimensions.LineWidth());
    renderVerticalLine(_x - Field_Dimensions::Current_Dimensions.Width() / 2.f,
                       _y + Field_Dimensions::Current_Dimensions.Length() / 2.f,
                       _x - Field_Dimensions::Current_Dimensions.Width() / 2.f,
                       _y - Field_Dimensions::Current_Dimensions.Length() / 2.f,
                       lineHeight,
                       Field_Dimensions::Current_Dimensions.LineWidth());
    renderHorizontalLine(
        _x + Field_Dimensions::Current_Dimensions.Width() / 2.f,
        _y + Field_Dimensions::Current_Dimensions.Length() / 2.f,
        _x - Field_Dimensions::Current_Dimensions.Width() / 2.f,
        _y + Field_Dimensions::Current_Dimensions.Length() / 2.f, lineHeight,
        Field_Dimensions::Current_Dimensions.LineWidth());
    renderHorizontalLine(
        _x + Field_Dimensions::Current_Dimensions.Width() / 2.f,
        _y - Field_Dimensions::Current_Dimensions.Length() / 2.f,
        _x - Field_Dimensions::Current_Dimensions.Width() / 2.f,
        _y - Field_Dimensions::Current_Dimensions.Length() / 2.f, lineHeight,
        Field_Dimensions::Current_Dimensions.LineWidth());

    // Center Line
    renderHorizontalLine(
        _x + Field_Dimensions::Current_Dimensions.Width() / 2.f, _y,
        _x - Field_Dimensions::Current_Dimensions.Width() / 2.f, _y, lineHeight,
        Field_Dimensions::Current_Dimensions.LineWidth());

    // Goal Arc //0 radians is in the Z direction (forward)
    renderArc(_x - (Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f),
              _y - Field_Dimensions::Current_Dimensions.Length() / 2.f,
              M_PI * 3 / 2.f, M_PI * 2.f, lineHeight,
              Field_Dimensions::Current_Dimensions.ArcRadius(),
              Field_Dimensions::Current_Dimensions.LineWidth() * 2, 90);
    renderArc(_x + (Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f),
              _y - Field_Dimensions::Current_Dimensions.Length() / 2.f, 0,
              M_PI / 2.f, lineHeight,
              Field_Dimensions::Current_Dimensions.ArcRadius(),
              Field_Dimensions::Current_Dimensions.LineWidth() * 2, 90);
    renderHorizontalLine(
        _x - Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f,
        _y - Field_Dimensions::Current_Dimensions.Length() / 2.f +
            Field_Dimensions::Current_Dimensions.ArcRadius(),
        _x + Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f,
        _y - Field_Dimensions::Current_Dimensions.Length() / 2.f +
            Field_Dimensions::Current_Dimensions.ArcRadius(),
        lineHeight, Field_Dimensions::Current_Dimensions.LineWidth());

    renderArc(_x - (Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f),
              _y + Field_Dimensions::Current_Dimensions.Length() / 2.f, M_PI,
              M_PI * 3 / 2.f, lineHeight,
              Field_Dimensions::Current_Dimensions.ArcRadius(),
              Field_Dimensions::Current_Dimensions.LineWidth() * 2, 90);
    renderArc(_x + (Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f),
              _y + Field_Dimensions::Current_Dimensions.Length() / 2.f,
              M_PI / 2.f, M_PI, lineHeight,
              Field_Dimensions::Current_Dimensions.ArcRadius(),
              Field_Dimensions::Current_Dimensions.LineWidth() * 2, 90);
    renderHorizontalLine(
        _x - Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f,
        _y + Field_Dimensions::Current_Dimensions.Length() / 2.f -
            Field_Dimensions::Current_Dimensions.ArcRadius(),
        _x + Field_Dimensions::Current_Dimensions.GoalFlat() / 2.f,
        _y + Field_Dimensions::Current_Dimensions.Length() / 2.f -
            Field_Dimensions::Current_Dimensions.ArcRadius(),
        lineHeight, Field_Dimensions::Current_Dimensions.LineWidth());

    // Goal Area
    glColor4f(1.0, 1.0, 0, 1.0);  // Yellow
    renderVerticalLine(
        _x - Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f,
        _y - Field_Dimensions::Current_Dimensions.Length() / 2.f,
        _x - Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f,
        _y - (Field_Dimensions::Current_Dimensions.Length() / 2.f +
              Field_Dimensions::Current_Dimensions.GoalDepth()),
        lineHeight, Field_Dimensions::Current_Dimensions.LineWidth());
    renderVerticalLine(
        _x + Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f,
        _y - Field_Dimensions::Current_Dimensions.Length() / 2.f,
        _x + Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f,
        _y - (Field_Dimensions::Current_Dimensions.Length() / 2.f +
              Field_Dimensions::Current_Dimensions.GoalDepth()),
        lineHeight, Field_Dimensions::Current_Dimensions.LineWidth());
    renderHorizontalLine(
        _x - Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f,
        _y - (Field_Dimensions::Current_Dimensions.Length() / 2.f +
              Field_Dimensions::Current_Dimensions.GoalDepth()),
        _x + Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f,
        _y - (Field_Dimensions::Current_Dimensions.Length() / 2.f +
              Field_Dimensions::Current_Dimensions.GoalDepth()),
        lineHeight, Field_Dimensions::Current_Dimensions.LineWidth());

    glColor4f(0, 0, 1.0, 1.0);  // Blue
    renderVerticalLine(
        _x - Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f,
        _y + Field_Dimensions::Current_Dimensions.Length() / 2.f,
        _x - Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f,
        _y + (Field_Dimensions::Current_Dimensions.Length() / 2.f +
              Field_Dimensions::Current_Dimensions.GoalDepth()),
        lineHeight, Field_Dimensions::Current_Dimensions.LineWidth());
    renderVerticalLine(
        _x + Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f,
        _y + Field_Dimensions::Current_Dimensions.Length() / 2.f,
        _x + Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f,
        _y + (Field_Dimensions::Current_Dimensions.Length() / 2.f +
              Field_Dimensions::Current_Dimensions.GoalDepth()),
        lineHeight, Field_Dimensions::Current_Dimensions.LineWidth());
    renderHorizontalLine(
        _x - Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f,
        _y + (Field_Dimensions::Current_Dimensions.Length() / 2.f +
              Field_Dimensions::Current_Dimensions.GoalDepth()),
        _x + Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f,
        _y + (Field_Dimensions::Current_Dimensions.Length() / 2.f +
              Field_Dimensions::Current_Dimensions.GoalDepth()),
        lineHeight, Field_Dimensions::Current_Dimensions.LineWidth());

    if (debug_mode & btIDebugDraw::DBG_DrawWireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
}

void Field::renderVerticalLine(float x1, float z1, float x2, float z2,
                               float height, float lineWidth) {
    glBegin(GL_POLYGON);
    glVertex3f(x1 - lineWidth, height, z1);
    glVertex3f(x1 + lineWidth, height, z1);
    glVertex3f(x2 + lineWidth, height, z2);
    glVertex3f(x2 - lineWidth, height, z2);
    glEnd();
}

void Field::renderHorizontalLine(float x1, float z1, float x2, float z2,
                                 float height, float lineWidth) {
    glBegin(GL_POLYGON);
    glVertex3f(x1, height, z1 - lineWidth);
    glVertex3f(x1, height, z1 + lineWidth);
    glVertex3f(x2, height, z2 + lineWidth);
    glVertex3f(x2, height, z2 - lineWidth);
    glEnd();
}

void Field::renderArc(float x, float z, float angle1, float angle2,
                      float height, float radius, float lineWidth,
                      int numPoints) {
    glBegin(GL_TRIANGLE_STRIP);
    float delta = (angle2 - angle1) / (float)numPoints;
    for (float theta = angle1; theta <= angle2; theta += delta) {
        glVertex3f(x + sin(theta) * (radius - lineWidth), height,
                   z + cos(theta) * (radius - lineWidth / 2.f));
        glVertex3f(x + sin(theta) * radius, height,
                   z + cos(theta) * (radius + lineWidth / 2.f));
    }
    glEnd();
}

void Field::reshapeBodies() {
    btCollisionShape* groundShape;  // = new btBoxShape(btVector3(50, 3, 50));
    //_simEngine->addCollisionShape(groundShape);
    btTransform tr;
    tr.setIdentity();

    // create walls
    const float halfWidth = 0.1 * scaling;
    const float halfHeight = 0.05 * scaling;

    btCollisionShape* longWallShape = new btBoxShape(
        btVector3(halfWidth, halfHeight,
                  Field_Dimensions::Current_Dimensions.FloorLength() / 2.f +
                      2 * halfWidth));
    btCollisionShape* wideWallShape = new btBoxShape(
        btVector3(Field_Dimensions::Current_Dimensions.FloorWidth() / 2.f,
                  halfHeight, halfWidth));
    longWallShape->setMargin(0.004 * scaling);
    wideWallShape->setMargin(0.004 * scaling);

    tr.setOrigin(btVector3(
        Field_Dimensions::Current_Dimensions.FloorWidth() / 2.f + halfWidth,
        halfHeight, 0));
    _wallBodies[0]->setCollisionShape(longWallShape);
    _wallBodies[0]->setMotionState(new btDefaultMotionState(tr));

    tr.setOrigin(btVector3(
        -Field_Dimensions::Current_Dimensions.FloorWidth() / 2.f - halfWidth,
        halfHeight, 0));
    _wallBodies[1]->setCollisionShape(longWallShape);
    _wallBodies[1]->setMotionState(new btDefaultMotionState(tr));

    tr.setOrigin(btVector3(
        0, halfHeight,
        Field_Dimensions::Current_Dimensions.FloorLength() / 2.f + halfWidth));
    _wallBodies[2]->setCollisionShape(wideWallShape);
    _wallBodies[2]->setMotionState(new btDefaultMotionState(tr));

    tr.setOrigin(btVector3(
        0, halfHeight,
        -Field_Dimensions::Current_Dimensions.FloorLength() / 2.f - halfWidth));
    _wallBodies[3]->setCollisionShape(wideWallShape);
    _wallBodies[3]->setMotionState(new btDefaultMotionState(tr));

    // color
    auto color = new btVector3(1, 1, 1);
    longWallShape->setUserPointer(color);
    wideWallShape->setUserPointer(color);

    // create goal walls for blue
    btBoxShape* goalBackShape = new btBoxShape(
        btVector3(Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f +
                      Sim_GoalWall_Width,
                  Sim_GoalWall_Height / 2.f, Sim_GoalWall_Width / 2.f));
    btBoxShape* goalSideShape = new btBoxShape(
        btVector3(Sim_GoalWall_Width / 2.f, Sim_GoalWall_Height / 2.f,
                  Field_Dimensions::Current_Dimensions.GoalDepth() / 2.f));
    goalBackShape->setMargin(0.004 * scaling);
    goalSideShape->setMargin(0.004 * scaling);

    btVector3 backPos =
        btVector3(0, Sim_GoalWall_Height / 2.f,
                  Field_Dimensions::Current_Dimensions.Length() / 2.f +
                      Sim_GoalWall_Width / 2.f +
                      Field_Dimensions::Current_Dimensions.GoalDepth());
    btVector3 sidePos =
        btVector3(Field_Dimensions::Current_Dimensions.GoalWidth() / 2.f +
                      Sim_GoalWall_Width / 2.f,
                  Sim_GoalWall_Height / 2.f,
                  Field_Dimensions::Current_Dimensions.Length() / 2.f +
                      Field_Dimensions::Current_Dimensions.GoalDepth() / 2.f);

    tr.setOrigin(backPos);
    _blueGoalBodies[0]->setCollisionShape(goalBackShape);
    _blueGoalBodies[0]->setMotionState(new btDefaultMotionState(tr));

    tr.setOrigin(sidePos);
    _blueGoalBodies[1]->setCollisionShape(goalSideShape);
    _blueGoalBodies[1]->setMotionState(new btDefaultMotionState(tr));

    sidePos.setX(-sidePos.x());
    tr.setOrigin(sidePos);
    _blueGoalBodies[2]->setCollisionShape(goalSideShape);
    _blueGoalBodies[2]->setMotionState(new btDefaultMotionState(tr));

    // color blue
    color = new btVector3(0, 0, 1);
    goalBackShape->setUserPointer(color);
    goalSideShape->setUserPointer(color);

    // create goal walls for yellow
    /// FIXME: Object colors are stored in the userPointer of btCollisionShapes.
    /// The side effect is that new collision shapes need to be created each
    /// time.
    /// Alternatives are to store them in btCollisionObject's userObjectPointer
    /// or
    /// plain figure out a better way to do this.
    goalBackShape = new btBoxShape(*goalBackShape);
    goalSideShape = new btBoxShape(*goalSideShape);
    goalBackShape->setMargin(0.004 * scaling);
    goalSideShape->setMargin(0.004 * scaling);

    backPos.setZ(-backPos.z());
    sidePos.setZ(-sidePos.z());

    tr.setOrigin(backPos);
    _yellowGoalBodies[0]->setCollisionShape(goalBackShape);
    _yellowGoalBodies[0]->setMotionState(new btDefaultMotionState(tr));

    tr.setOrigin(sidePos);
    _yellowGoalBodies[1]->setCollisionShape(goalSideShape);
    _yellowGoalBodies[1]->setMotionState(new btDefaultMotionState(tr));

    sidePos.setX(-sidePos.x());
    tr.setOrigin(sidePos);
    _yellowGoalBodies[2]->setCollisionShape(goalSideShape);
    _yellowGoalBodies[2]->setMotionState(new btDefaultMotionState(tr));

    // color blue
    color = new btVector3(1, 1, 0.5f);
    goalBackShape->setUserPointer(color);
    goalSideShape->setUserPointer(color);
}