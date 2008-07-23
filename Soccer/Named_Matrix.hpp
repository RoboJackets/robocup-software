#ifndef _NAMED_MATRIX_H_
#define _NAMED_MATRIX_H_

#include <Geometry/TransformMatrix.hpp>
#include <string>
#include <map>

#include "Named_Object.hpp"

class Named_Matrix: public Named_Object<Named_Matrix>, public Geometry::TransformMatrix
{
public:
    Named_Matrix(const char *name): Named_Object<Named_Matrix>(name) {}
    
    void operator=(const Geometry::TransformMatrix &other)
    {
        const float *om = other.m();
        _m[0] = om[0];
        _m[1] = om[1];
        _m[2] = om[2];
        _m[3] = om[3];
        _m[4] = om[4];
        _m[5] = om[5];
    }
};

#endif // _NAMED_MATRIX_H_
