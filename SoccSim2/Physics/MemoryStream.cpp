#include "MemoryStream.hpp"

MemoryStream::MemoryStream()
{
    _readLoc = 0;
}

MemoryStream::~MemoryStream()
{
}

// Loading API
NxU8 MemoryStream::readByte() const
{
    NxU8 b = read<NxU8>();
    //printf("<< BYTE %x\n", b);
    
    return b;
}

NxU16 MemoryStream::readWord() const
{
    NxU16 w = read<NxU16>();
    //printf("<< WORD %x\n", w);
    
    return w;
}

NxU32 MemoryStream::readDword() const
{
    NxU32 d = read<NxU32>();
    //printf("<< DWORD %x\n", d);
    
    return d;
}

float MemoryStream::readFloat() const
{
    float f = read<float>();
    //printf("<< FLOAT %f\n", f);
    
    return f;
}

double MemoryStream::readDouble() const
{
    double f = read<double>();
    //printf("<< DOUBLE %f\n", f);
    
    return f;
}

void MemoryStream::readBuffer(void* buffer, NxU32 size) const
{
    uint8_t* bytes = (uint8_t*)buffer;
 
    memcpy (bytes, &_data[_readLoc], size);
    _readLoc += size;
    
#if 0
    printf("Read: %d\n", size);
    printf("<< ");
    for (NxU32 i=0 ; i<size ; ++i)
    {
        printf("%x ", bytes[i]);
    }
    printf("\n");
#endif
}

// Saving API
NxStream& MemoryStream::storeByte(NxU8 b)
{
    storeBuffer(&b, sizeof(b));
    return *this;
}

NxStream& MemoryStream::storeWord(NxU16 w)
{
    storeBuffer(&w, sizeof(w));
    return *this;
}

NxStream& MemoryStream::storeDword(NxU32 d)
{
    storeBuffer(&d, sizeof(d));
    return *this;   
}

NxStream& MemoryStream::storeFloat(NxReal f)
{
    storeBuffer(&f, sizeof(f));
    return *this;
}

NxStream& MemoryStream::storeDouble(NxF64 f)
{
    storeBuffer(&f, sizeof(f));
    return *this;
}

NxStream& MemoryStream::storeBuffer(const void* buffer, NxU32 size)
{
    const unsigned int length = _data.size();
    _data.resize(length + size);
    memcpy(&_data[length], buffer, size);
    
#if 0
    printf(">> BUFF ");
    for (NxU32 i=0 ; i<size ; ++i)
    {
        printf("%x ", _data[length + i]);
    }
    printf("\n");
#endif
    
    return *this;
}
