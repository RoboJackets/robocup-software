#ifndef _SERIALIZATION_HPP_
#define _SERIALIZATION_HPP_

#include <stdint.h>
#include <vector>

namespace Serialization
{
    class Buffer
    {
    public:
        void write(uint8_t value);
        void write(int8_t value);
        void write(uint16_t value);
        void write(int16_t value);
        void write(uint32_t value);
        void write(int32_t value);
        void write(uint64_t value);
        void write(int64_t value);
        void write(float value);
        void write(double value);
        
        template<typename S, typename T>
        void writeClassArray(std::vector<T> &x)
        {
            write((S)x.size());
            for (S i = 0; i < x.size(); ++i)
            {
                x[i].write(*this);
            }
        }
        
        template<typename S, typename T>
        void writePrimitiveArray(std::vector<T> &x)
        {
            write((S)x.size());
            for (S i = 0; i < x.size(); ++i)
            {
                write(x[i]);
            }
        }
        
        template<typename S, typename T, unsigned int N>
        void writePrimitiveArray(T (&x)[N])
        {
            write((S)N);
            for (S i = 0; i < N; ++i)
            {
                write(x[i]);
            }
        }
        
        template<typename S, class T, unsigned int N>
        void writeClassArray(T (&x)[N])
        {
            write((S)N);
            for (S i = 0; i < N; ++i)
            {
                x[i].write(*this);
            }
        }
    };
}

#endif // _SERIALIZATION_HPP_
