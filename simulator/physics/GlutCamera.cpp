#include "GlutCamera.hpp"
#include <physics/SimEngine.hpp>
#include <physics/Robot.hpp>
#include "GLDebugFont.h"

using namespace std;

GlutCamera::GlutCamera(SimEngine* engine)
    : _mode(0),
      _vehicle(nullptr),
      _cameraDistance(1.0 * scaling),
      _ele(20.f),
      _azi(0.f),
      _cameraPosition(0.f, 0.f, 0.f),
      _cameraTargetPosition(0.f, 0.f, 0.f),
      _scaleBottom(0.5f),
      _scaleFactor(2.f),
      _cameraUp(0, 1, 0),
      _forwardAxis(2),
      _glutScreenWidth(0),
      _glutScreenHeight(0),
      _frustumZNear(.1f),
      _frustumZFar(1000.f),
      _ortho(0),
      _simEngine(engine) {
    _shapeDrawer = new GL_ShapeDrawer();
    _shapeDrawer->enableTexture(false);
    _enableshadows = false;

    setCameraMode(SideLine);
}

GlutCamera::~GlutCamera() {
    if (_shapeDrawer) delete _shapeDrawer;
}

void GlutCamera::reshape(int w, int h) {
    GLDebugResetFont(w, h);

    _glutScreenWidth = w;
    _glutScreenHeight = h;

    glViewport(0, 0, w, h);
    updateCamera();
}

void GlutCamera::myinit(void) {
    GLfloat light_ambient[] = {btScalar(0.2), btScalar(0.2), btScalar(0.2),
                               btScalar(1.0)};
    GLfloat light_diffuse[] = {btScalar(1.0), btScalar(1.0), btScalar(1.0),
                               btScalar(1.0)};
    GLfloat light_specular[] = {btScalar(1.0), btScalar(1.0), btScalar(1.0),
                                btScalar(1.0)};
    /*	light_position is NOT default value	*/
    GLfloat light_position0[] = {btScalar(1.0), btScalar(10.0), btScalar(1.0),
                                 btScalar(0.0)};
    GLfloat light_position1[] = {btScalar(-1.0), btScalar(-10.0),
                                 btScalar(-1.0), btScalar(0.0)};

    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

    glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);

    glShadeModel(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glClearColor(btScalar(0.7), btScalar(0.7), btScalar(0.7), btScalar(0));

    //  glEnable(GL_CULL_FACE);
    //  glCullFace(GL_BACK);
}

void GlutCamera::setCameraMode(int mode) {
    _mode = mode;
    if (_mode & SideLine) {
        _ele = 40;
        _azi = 90;
        _cameraTargetPosition = btVector3(-10, 0, 0);
        _cameraDistance = 30;
    }
    if (_mode & Overhead) {
        _ele = 89.5;
        _azi = 90;
        _cameraTargetPosition = btVector3(0, 0, 0);
        _cameraDistance = 30;
    }
    if (_mode & BehindYellowGoal) {
        _cameraDistance = Field_Dimensions::Current_Dimensions.Length() / 4.f;
        _cameraTargetPosition = btVector3(
            0, 0, -Field_Dimensions::Current_Dimensions.Length() / 2.f);
        _ele = 50;
        _azi = 0;
    }
    if (_mode & BehindBlueGoal) {
        _cameraDistance = Field_Dimensions::Current_Dimensions.Length() / 4.f;
        _cameraTargetPosition = btVector3(
            0, 0, Field_Dimensions::Current_Dimensions.Length() / 2.f);
        _ele = 50;
        _azi = 180;
    }
    if (_mode & TrackVehicle && _vehicle) {
        _ele = 20;
        _cameraDistance = 10;
    }
    if (_mode & BehindVehicle && _vehicle) {
        // do later
    }
    if (_mode & FrontOfVehicle && _vehicle) {
        // do later
    }
    if (_mode & Orthogonal) {
    }
    if (_mode & FreeMove) {
    }
    if (_mode & Reset) {
        // do nothing
    }
}

void GlutCamera::updateCamera() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    btScalar rele = _ele * btScalar(0.01745329251994329547);  // rads per deg
    btScalar razi = _azi * btScalar(0.01745329251994329547);  // rads per deg

    if ((_mode & TrackVehicle) && _vehicle) {
        btTransform trans;
        _vehicle->getRigidBody()->getMotionState()->getWorldTransform(trans);
        _cameraTargetPosition = trans.getOrigin();
        if (_mode & BehindVehicle) {
            razi = trans.getRotation().getAxis().getY() *
                   trans.getRotation().getAngle();
        }
        if (_mode & FrontOfVehicle) {
            razi = 3.14159265 +
                   trans.getRotation().getAxis().getY() *
                       trans.getRotation().getAngle();
        }
    }

    btQuaternion rot(_cameraUp, razi);

    btVector3 eyePos(0, 0, 0);
    eyePos[_forwardAxis] = -_cameraDistance;

    btVector3 forward(eyePos[0], eyePos[1], eyePos[2]);
    if (forward.length2() < SIMD_EPSILON) {
        forward.setValue(1.f, 0.f, 0.f);
    }
    btVector3 right = _cameraUp.cross(forward);
    btQuaternion roll(right, -rele);

    eyePos = btMatrix3x3(rot) * btMatrix3x3(roll) * eyePos;

    _cameraPosition[0] = eyePos.getX();
    _cameraPosition[1] = eyePos.getY();
    _cameraPosition[2] = eyePos.getZ();
    _cameraPosition += _cameraTargetPosition;

    if (_glutScreenWidth == 0 && _glutScreenHeight == 0) return;

    btScalar aspect;
    btVector3 extents;

    aspect = _glutScreenWidth / (btScalar)_glutScreenHeight;
    extents.setValue(aspect * 1.0f, 1.0f, 0);

    if (_ortho) {
        // reset matrix
        glLoadIdentity();

        extents *= _cameraDistance;
        btVector3 lower = _cameraTargetPosition - extents;
        btVector3 upper = _cameraTargetPosition + extents;
        // gluOrtho2D(lower.x, upper.x, lower.y, upper.y);
        glOrtho(lower.getX(), upper.getX(), lower.getY(), upper.getY(), -1000,
                1000);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        // glTranslatef(100,210,0);
    } else {
        // 	glFrustum (-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);
        glFrustum(-aspect * _frustumZNear, aspect * _frustumZNear,
                  -_frustumZNear, _frustumZNear, _frustumZNear, _frustumZFar);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(_cameraPosition[0], _cameraPosition[1], _cameraPosition[2],
                  _cameraTargetPosition[0], _cameraTargetPosition[1],
                  _cameraTargetPosition[2], _cameraUp.getX(), _cameraUp.getY(),
                  _cameraUp.getZ());
    }
}

//
btVector3 GlutCamera::getRayTo(int x, int y) {
    if (_ortho) {
        btScalar aspect;
        btVector3 extents;
        aspect = _glutScreenWidth / (btScalar)_glutScreenHeight;
        extents.setValue(aspect * 1.0f, 1.0f, 0);

        extents *= _cameraDistance;
        btVector3 lower = _cameraTargetPosition - extents;
        btVector3 upper = _cameraTargetPosition + extents;

        btScalar u = x / btScalar(_glutScreenWidth);
        btScalar v = (_glutScreenHeight - y) / btScalar(_glutScreenHeight);

        btVector3 p(0, 0, 0);
        p.setValue((1.0f - u) * lower.getX() + u * upper.getX(),
                   (1.0f - v) * lower.getY() + v * upper.getY(),
                   _cameraTargetPosition.getZ());
        return p;
    }

    float top = 1.f;
    float bottom = -1.f;
    float nearPlane = 1.f;
    float tanFov = (top - bottom) * 0.5f / nearPlane;
    float fov = btScalar(2.0) * btAtan(tanFov);

    btVector3 rayFrom = getCameraPosition();
    btVector3 rayForward = (getCameraTargetPosition() - getCameraPosition());
    rayForward.normalize();
    float farPlane = 10000.f;
    rayForward *= farPlane;

    btVector3 rightOffset;
    btVector3 vertical = _cameraUp;

    btVector3 hor;
    hor = rayForward.cross(vertical);
    hor.normalize();
    vertical = hor.cross(rayForward);
    vertical.normalize();

    float tanfov = tanf(0.5f * fov);

    hor *= 2.f * farPlane * tanfov;
    vertical *= 2.f * farPlane * tanfov;

    btScalar aspect;

    aspect = _glutScreenWidth / (btScalar)_glutScreenHeight;

    hor *= aspect;

    btVector3 rayToCenter = rayFrom + rayForward;
    btVector3 dHor = hor * 1.f / float(_glutScreenWidth);
    btVector3 dVert = vertical * 1.f / float(_glutScreenHeight);

    btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
    rayTo += btScalar(x) * dHor;
    rayTo -= btScalar(y) * dVert;
    return rayTo;
}

void GlutCamera::swapBuffers() { glutSwapBuffers(); }

// See http://www.lighthouse3d.com/opengl/glut/index.php?bmpfontortho
void GlutCamera::setOrthographicProjection() {
    // switch to projection mode
    glMatrixMode(GL_PROJECTION);

    // save previous matrix which contains the settings for the perspective
    // projection
    glPushMatrix();
    // reset matrix
    glLoadIdentity();
    // set a 2D orthographic projection
    gluOrtho2D(0, _glutScreenWidth, 0, _glutScreenHeight);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // invert the y axis, down is positive
    glScalef(1, -1, 1);
    // mover the origin from the bottom left corner to the upper left corner
    glTranslatef(btScalar(0), btScalar(-_glutScreenHeight), btScalar(0));
}

void GlutCamera::resetPerspectiveProjection() {
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    updateCamera();
}

void GlutCamera::displayProfileString(int xOffset, int yStart, char* message) {
    glRasterPos3f(btScalar(xOffset), btScalar(yStart), btScalar(0));
    GLDebugDrawString(xOffset, yStart, message);
}

void GlutCamera::renderscene(int pass, int debugMode) {
    // throw runtime_error("Breaking at renderscene");
    btScalar m[16];
    btMatrix3x3 rot;
    rot.setIdentity();
    const int numObjects =
        _simEngine->dynamicsWorld()->getNumCollisionObjects();
    btVector3 wireColor(1, 0, 0);
    for (int i = 0; i < numObjects; i++) {
        btCollisionObject* colObj =
            _simEngine->dynamicsWorld()->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(colObj);
        if (body && body->getMotionState()) {
            btDefaultMotionState* myMotionState =
                (btDefaultMotionState*)body->getMotionState();
            myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
            rot = myMotionState->m_graphicsWorldTrans.getBasis();
        } else {
            colObj->getWorldTransform().getOpenGLMatrix(m);
            rot = colObj->getWorldTransform().getBasis();
        }
        btVector3 wireColor(1.f, 1.0f, 0.5f);              // wants deactivation
        if (i & 1) wireColor = btVector3(0.f, 0.0f, 1.f);  //
        /// color differently for active, sleeping, wantsdeactivation states
        if (colObj->getActivationState() == 1)  // active
        {
            if (i & 1) {
                wireColor += btVector3(1.f, 0.f, 0.f);
            } else {
                wireColor += btVector3(.5f, 0.f, 0.f);
            }
        }
        if (colObj->getActivationState() == 2)  // ISLAND_SLEEPING
        {
            if (i & 1) {
                wireColor += btVector3(0.f, 1.f, 0.f);
            } else {
                wireColor += btVector3(0.f, 0.5f, 0.f);
            }
        }
        // color collision shapes
        if (colObj->getCollisionShape()->getUserPointer()) {
            wireColor =
                *((btVector3*)(colObj->getCollisionShape()->getUserPointer()));
        }

        btVector3 aabbMin, aabbMax;
        _simEngine->dynamicsWorld()->getBroadphase()->getBroadphaseAabb(
            aabbMin, aabbMax);

        aabbMin -= btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
        aabbMax += btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
        // 	printf("aabbMin=(%f,%f,%f)\n",aabbMin.getX(),aabbMin.getY(),aabbMin.getZ());
        // 	printf("aabbMax=(%f,%f,%f)\n",aabbMax.getX(),aabbMax.getY(),aabbMax.getZ());
        // 	_dynamicsWorld->getDebugDrawer()->drawAabb(aabbMin,aabbMax,btVector3(1,1,1));

        if (!(debugMode & btIDebugDraw::DBG_DrawWireframe)) {
            switch (pass) {
                case 0:
                    _shapeDrawer->drawOpenGL(m, colObj->getCollisionShape(),
                                             wireColor, debugMode, aabbMin,
                                             aabbMax);
                    break;
                case 1:
                    _shapeDrawer->drawShadow(m, _sundirection * rot,
                                             colObj->getCollisionShape(),
                                             aabbMin, aabbMax);
                    break;
                case 2:
                    _shapeDrawer->drawOpenGL(m, colObj->getCollisionShape(),
                                             wireColor * btScalar(0.3), 0,
                                             aabbMin, aabbMax);
                    break;
            }
        }
    }
}

void GlutCamera::renderme(int debugMode) {
    myinit();
    updateCamera();
    if (_simEngine->dynamicsWorld()) {
        if (_enableshadows) {
            glClear(GL_STENCIL_BUFFER_BIT);
            glEnable(GL_CULL_FACE);
            renderscene(0, debugMode);

            glDisable(GL_LIGHTING);
            glDepthMask(GL_FALSE);
            glDepthFunc(GL_LEQUAL);
            glEnable(GL_STENCIL_TEST);
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
            glStencilFunc(GL_ALWAYS, 1, 0xFFFFFFFFL);
            glFrontFace(GL_CCW);
            glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
            renderscene(1, debugMode);
            glFrontFace(GL_CW);
            glStencilOp(GL_KEEP, GL_KEEP, GL_DECR);
            renderscene(1, debugMode);
            glFrontFace(GL_CCW);

            glPolygonMode(GL_FRONT, GL_FILL);
            glPolygonMode(GL_BACK, GL_FILL);
            glShadeModel(GL_SMOOTH);
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glEnable(GL_LIGHTING);
            glDepthMask(GL_TRUE);
            glCullFace(GL_BACK);
            glFrontFace(GL_CCW);
            glEnable(GL_CULL_FACE);
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

            glDepthFunc(GL_LEQUAL);
            glStencilFunc(GL_NOTEQUAL, 0, 0xFFFFFFFFL);
            glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
            glDisable(GL_LIGHTING);
            renderscene(2, debugMode);
            glEnable(GL_LIGHTING);
            glDepthFunc(GL_LESS);
            glDisable(GL_STENCIL_TEST);
            glDisable(GL_CULL_FACE);
        } else {
            glDisable(GL_CULL_FACE);
            renderscene(0, debugMode);
        }
    }

    updateCamera();
}

void GlutCamera::chaseCamera() {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    btScalar aspect = _glutScreenWidth / (btScalar)_glutScreenHeight;
    glFrustum(-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt(_cameraPosition[0], _cameraPosition[1], _cameraPosition[2],
              _cameraTargetPosition[0], _cameraTargetPosition[1],
              _cameraTargetPosition[2], _cameraUp.getX(), _cameraUp.getY(),
              _cameraUp.getZ());
}
