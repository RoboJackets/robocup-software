#include <stdio.h>

namespace Geometry
{
    class Point2d
    {
    public:
        Point2d()
        {
            x = 1;
            y = 2;
        }
        
        float x, y;
    };
};

#include "test.hpp"

int main()
{
    FILE *fp = fopen("packet.bin", "w+b");
    Serialization::FileBuffer fb(fp);
    
    LogFrame f;
    f.intArray.push_back(5);
    f.intArray.push_back(7);
    f.intArray.push_back(6);
    f.e_array_var.push_back(LogFrame::B);
    f.someText = "abc";
    Serialization::WriteBuffer &out = fb;
    out & f;
    
    fseek(fp, SEEK_SET, 0);
    LogFrame r;
    Serialization::ReadBuffer &in = fb;
    in & r;
    printf("intArray: %d items\n", r.intArray.size());
    for (unsigned int i = 0; i < r.intArray.size(); ++i)
    {
        printf("    %d: %d\n", i, r.intArray[i]);
    }
    printf("Text: \"%s\"\n", r.someText.c_str());
    
    return 0;
}
