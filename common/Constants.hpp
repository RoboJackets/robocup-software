#ifndef _CONSTANTS_HPP_
#define _CONSTANTS_HPP_

/// all distances in meters
/// all times in seconds
/// all weights in killograms

namespace Constants
{
    namespace Ball
    {
        const float Diameter = 0.043f;
        const float Radius = Diameter/2.0f;
        const float Mass = 0.048f;
    }
    
    namespace Field
    {
        const float Length = 6.1f;
        const float Width = 4.2f;
        const float Border = .25f;
        
        const float LineWidth = 0.01f;
    }
    
    namespace Floor
    {
        const float Length = Field::Length + 2.0 * Field::Border;
        const float Width = Field::Width + 2.0 * Field::Border;
    }
    
    namespace Robot
    {
        const float Diameter = .180f;
        const float Radius = Diameter/2.0f;
        const float Height = .150f;
    }
}

#endif /* _CONSTANTS_HPP_ */
