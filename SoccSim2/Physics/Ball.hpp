#ifndef _BALL_HPP
#define _BALL_HPP

#include "Entity.hpp"

class Ball : public Entity
{
    public:
        Ball(NxScene& scene);
        ~Ball();
        
        virtual void paint() const;
        
    private:
        /** glu graphics object for the ball */
        GLUquadric* _quadric;
};

#endif /* _BALL_HPP */
