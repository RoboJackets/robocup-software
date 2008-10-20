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

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t ready = PTHREAD_COND_INITIALIZER;

void *receive_thread(void *arg)
{
    Serialization::Receiver r("224.1.2.3", 1234, 1024);
    
    pthread_mutex_lock(&mutex);
    pthread_cond_signal(&ready);
    pthread_mutex_unlock(&mutex);
    
    Packet::LogFrame f;
    r.receive(f);
    
    printf("intArray: %d items\n", (int)f.intArray.size());
    for (unsigned int i = 0; i < f.intArray.size(); ++i)
    {
        printf("\t%d: %d\n", i, f.intArray[i]);
    }
    printf("Text: \"%s\"\n", f.someText.c_str());
    
    return 0;
}

int main()
{
    pthread_t t;
    pthread_mutex_lock(&mutex);
    pthread_create(&t, 0, receive_thread, 0);
    pthread_cond_wait(&ready, &mutex);
    
    Serialization::Sender s("224.1.2.3", 1234);
    
    Packet::LogFrame f;
    f.intArray.push_back(5);
    f.intArray.push_back(7);
    f.intArray.push_back(6);
    f.e_array_var.push_back(Packet::LogFrame::B);
    f.someText = "abc";
    s.send(f);
    
    pthread_join(t, 0);
    
    return 0;
}
