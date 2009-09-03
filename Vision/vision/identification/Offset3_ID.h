#pragma once

#include "Vector_ID.h"

namespace Vision
{
    class Offset3_ID: public Vector_ID
    {
    public:
        Offset3_ID(Process *process, Color center_color, const Vector_Pattern pattern);
        
        virtual void run();
    };
}
