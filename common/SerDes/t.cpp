#include <stdio.h>

using namespace std;

#if 0
namespace Geometry2d
{
    class Point
    {
    public:
        Point()
        {
            x = 1;
            y = 2;
        }
        
        float x, y;
    };
};
#endif

#include "test.hpp"

int main()
{
    Serialization::MemoryBuffer buf;
    Serialization::ReadBuffer &r = buf;
    Serialization::WriteBuffer &w = buf;
    
    Packet::LogFrame f1;
    f1.intArray.push_back(5);
    f1.intArray.push_back(7);
    f1.intArray.push_back(6);
    f1.e_array_var.push_back(Packet::LogFrame::B);
    f1.someText = "abc";
    f1.team = Blue;
    f1.otherTeam = Blue;
    w & f1;

    Packet::LogFrame f2;
    r & f2;

    printf("intArray: %d items\n", (int)f2.intArray.size());
    for (unsigned int i = 0; i < f2.intArray.size(); ++i)
    {
        printf("\t%d: %d\n", i, f2.intArray[i]);
    }
    printf("Text: \"%s\"\n", f2.someText.c_str());
    printf("Bools: %d %d\n", f2.flag, f2.flag_default);
    printf("Teams %d %d\n", f2.team, f2.otherTeam);
    
    return 0;
}
