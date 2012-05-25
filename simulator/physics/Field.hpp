#pragma once

#include "Entity.hpp"
#include <GL/glut.h>


class Field : public Entity
{
    public:
        Field(Environment* env);
        virtual ~Field();
        
        //Translates the origin of the field
        void position(float x, float y) { _x = x; _y = y; }
        
        void renderField();

    private:
        float _x;
        float _y;

        void renderVerticalLine(float x1, float z1, float x2, float z2, float height, float lineWidth);
        void renderHorizontalLine(float x1, float z1, float x2, float z2, float height, float lineWidth);
        void renderArc(float x, float z, float angle1, float angle2, float height, float radius, float lineWidth, int numPoints);
};
