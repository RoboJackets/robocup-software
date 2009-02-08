#ifndef _FIELD_HPP
#define _FIELD_HPP

#include "Entity.hpp"

class Field : public Entity
{
    public:
        Field(NxScene& scene);
        ~Field();
        
        //does nothing for the field
        virtual void position(float x, float y) {};
        
    private:
};


#endif /* _FIELD_HPP */
