#pragma once

#include "Entity.hpp"

class Field : public Entity
{
    public:
        Field(Env* env);
        virtual ~Field();
        
        //does nothing for the field
        virtual void position(float x, float y) {};
        
    private:
};
