#include <stdio.h>

#include "test.hpp"

void Serialization::WriteBuffer::operator&(uint8_t x)
{
    printf("uint8_t %d\n", x);
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
    buf & f;
    
    return 0;
}
