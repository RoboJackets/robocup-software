#ifndef _VECTOR_ID_H_
#define _VECTOR_ID_H_

#include "Dot_ID.h"

namespace Vision
{
    class Vector_Pattern: public Pattern
    {
    public:
        class Pair
        {
        public:
            Pair(unsigned int first, unsigned int second, float a)
            {
                i0 = first;
                i1 = second;
                angle = a;
            }
            
            // Dot indices
            unsigned int i0, i1;
            
            // Rotate by this
            float angle;
        };
        
        Vector_Pattern()
        {
            min_dots = 0;
        }
        
        Vector_Pattern(float a, const Geometry::Point2d &c,
                       const std::vector<Color_Sequence> &s,
                       const std::vector<Pair> &p):
            Pattern(a, c, s), pairs(p)
        {
            min_dots = 0;
            for (unsigned int i = 0; i < pairs.size(); ++i)
            {
                if (pairs[i].i0 > min_dots)
                {
                    min_dots = pairs[i].i0;
                }
                if (pairs[i].i1 > min_dots)
                {
                    min_dots = pairs[i].i1;
                }
            }
            ++min_dots;
        }
        
        std::vector<Pair> pairs;
        
        // Minimum number of dots required by this pattern
        unsigned int min_dots;
    };
    
    class Vector_ID: public Processor, public Dot_ID
    {
    public:
        Vector_ID(Process *process, Color center_color, const Vector_Pattern &pattern);
        
        virtual void run();
        
        static Vector_Pattern ZJUNlict, Botnia, Strive, CMU, GaTech_Old, GaTech, Unknown2, BSmart, PlasmaZ;
        static Vector_Pattern Robodragons, Khainui, Nav;
        
    protected:
        const Vector_Pattern &_pattern;
        bool _reverse_angle;
        
        void identify(Group *center_group);
    };
};

#endif // _VECTOR_ID_H_
