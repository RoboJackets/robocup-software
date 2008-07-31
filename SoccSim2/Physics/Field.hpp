#ifndef _FIELD_HPP
#define _FIELD_HPP

#include "Entity.hpp"

class Field : public Entity
{
    public:
        Field(NxScene& scene);
        ~Field();
        
        void paint() const;
    private:
};


#endif /* _FIELD_HPP */
