#include <stdio.h>

#include "Sender.hpp"
#include "Receiver.hpp"

using namespace std;

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

void *receive_thread(void *arg)
{
    Receiver r(1234);
    
    Serialization::MemoryBuffer fb;
    fb.data.resize(1024);
    r.receive(fb.data);
#if 1
    printf("%d: ", fb.data.size());
    for (unsigned int i = 0; i < fb.data.size(); ++i)
    {
        printf("%02x ", fb.data[i]);
    }
    printf("\n");
#endif
    
    LogFrame f;
    Serialization::ReadBuffer &in = fb;
    in & f;
#if 0
    printf("intArray: %d items\n", (int)f.intArray.size());
    for (unsigned int i = 0; i < f.intArray.size(); ++i)
    {
        printf("\t%d: %d\n", i, f.intArray[i]);
    }
    printf("Text: \"%s\"\n", f.someText.c_str());
#endif
    
    return 0;
}

int main()
{
    pthread_t t;
    pthread_create(&t, 0, receive_thread, 0);
    
    Sender s("localhost", 1234);
    
    LogFrame f;
//    f.intArray.push_back(5);
//    f.intArray.push_back(7);
//    f.intArray.push_back(6);
//    f.e_array_var.push_back(LogFrame::B);
//    f.someText = "abc";
    s.send(f);
    
    Serialization::MemoryBuffer fb;
    Serialization::WriteBuffer &out = fb;
    out & f;
    printf("ser %d\n", fb.data.size());
    
    return 0;
}
