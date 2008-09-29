#include <stdio.h>

#include "test.hpp"

void Serialization::WriteBuffer::operator&(uint8_t x)
{
    printf("uint8_t %d\n", x);
}

void Serialization::WriteBuffer::operator&(int8_t x)
{
    printf("int8_t %d\n", x);
}

void Serialization::WriteBuffer::operator&(int32_t x)
{
    printf("int32_t %d\n", x);
}

void Serialization::WriteBuffer::operator&(uint32_t x)
{
    printf("uint32_t %d\n", x);
}

void Serialization::WriteBuffer::operator&(uint64_t x)
{
    printf("uint64_t %lld\n", x);
}

void Serialization::WriteBuffer::operator&(float x)
{
    printf("float %f\n", x);
}

int main()
{
    Serialization::WriteBuffer buf;
    
    LogFrame f;
    f.intArray.push_back(5);
    f.intArray.push_back(7);
    f.intArray.push_back(6);
    f.e_array_var.push_back(LogFrame::B);
    f.someText = "abc";
    buf & f;
    
    return 0;
}
