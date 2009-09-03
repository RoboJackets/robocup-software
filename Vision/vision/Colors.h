#pragma once

namespace Vision
{
    // Colors
    typedef enum
    {
        Orange,     // 0
        Blue,       // 1
        Yellow,     // 2
        Green,      // 3
        Pink,       // 4
        White,      // 5
        Unused6,    // 6
        Unused7,    // 7
        
        Num_Colors  // MUST BE 8 (so the type can be uint8_t)
    } Color;

    static const int In_Use_Colors = 6;

    extern const char *color_name[Num_Colors];
}
