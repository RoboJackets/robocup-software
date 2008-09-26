#ifndef _SERIALIZATION_HPP_
#define _SERIALIZATION_HPP_

#include <stdint.h>
#include <vector>

namespace Serialization
{
    class WriteBuffer
    {
    public:
        void operator&(uint8_t value);
        void operator&(int8_t value);
        void operator&(uint16_t value);
        void operator&(int16_t value);
        void operator&(uint32_t value);
        void operator&(int32_t value);
        void operator&(uint64_t value);
        void operator&(int64_t value);
        void operator&(float value);
        void operator&(double value);
        
        template<typename S, typename T>
        void arrayVariable(std::vector<T> &x)
        {
            *this & (S)x.size();
            // Always use unsigned int here so at least the loop
            // works correctly if sizetype is too small.
            for (unsigned int i = 0; i < x.size(); ++i)
            {
                *this & x[i];
            }
        }
        
        template<typename T, unsigned int N>
        void arrayFixed(T (&x)[N])
        {
            for (unsigned int i = 0; i < N; ++i)
            {
                *this & x[i];
            }
        }
    };
    
    class ReadBuffer
    {
    public:
        void operator&(uint8_t value);
        void operator&(int8_t value);
        void operator&(uint16_t value);
        void operator&(int16_t value);
        void operator&(uint32_t value);
        void operator&(int32_t value);
        void operator&(uint64_t value);
        void operator&(int64_t value);
        void operator&(float value);
        void operator&(double value);
        
        template<typename S, typename T>
        void arrayVariable(std::vector<T> &x)
        {
            S size;
            *this & size;
            x.resize(size);
            for (S i = 0; i < size; ++i)
            {
                *this & x[i];
            }
        }
        
        template<typename T, unsigned int N>
        void arrayFixed(T (&x)[N])
        {
            for (unsigned int i = 0; i < N; ++i)
            {
                *this & x[i];
            }
        }
    };
}

#endif // _SERIALIZATION_HPP_
