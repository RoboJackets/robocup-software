#pragma once

#include <stdint.h>
#include <vector>
#include <string>

namespace Serialization
{
    // The & operator is used for single member serialization.
    //
    // Arrays are serialized with one of four templated functions.
    //
    // Variable and fixed-size arrays are handled differently because
    // variable-size arrays need to serialize their size first.
    //
    // Arrays of enums are handled differently from arrays of other types
    // because enumerated types are always int in C++, but the XML may specify
    // another integer type.  In this case one of the array*Cast functions
    // is used, which will convert each element to the desired type.
    // The array*Cast functions are not used for all arrays because that would
    // require a lot of redundant typenames in the generated code.
    
    class WriteBuffer
    {
    public:
        virtual ~WriteBuffer() {}
        
        virtual void writeByte(uint8_t value) = 0;
        
        void operator&(uint8_t value) { add(value); }
        void operator&(int8_t value) { add(value); }
        void operator&(uint16_t value) { add(value); }
        void operator&(int16_t value) { add(value); }
        void operator&(uint32_t value) { add(value); }
        void operator&(int32_t value) { add(value); }
        void operator&(uint64_t value) { add(value); }
        void operator&(int64_t value) { add(value); }
        void operator&(float value) { add(value); }
        void operator&(double value) { add(value); }

        void operator&(bool value)
        {
            uint8_t x = value ? 1 : 0;
            add(x);
        }
        
        template<typename T>
        void add(T &value)
        {
            for (unsigned int i = 0; i < sizeof(T); ++i)
            {
                writeByte(((uint8_t *)&value)[i]);
            }
        }
        
        void operator&(std::string &value)
        {
            *this & (uint32_t)value.size();
            for (unsigned int i = 0; i  < value.size(); ++i)
            {
                *this & (int8_t)value[i];
            }
        }
        
        template<typename C, typename T>
        void memberCast(T &x)
        {
            C cx = (C)x;
            *this & cx;
        }
        
        template<typename S, typename T>
        void arrayVariable(std::vector<T> &x)
        {
            *this & (S)x.size();
            // Always use unsigned int here so the loop
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
        
        template<typename S, typename C, typename T>
        void arrayVariableCast(std::vector<T> &x)
        {
            *this & (S)x.size();
            // Always use unsigned int here so the loop
            // works correctly if sizetype is too small.
            for (unsigned int i = 0; i < x.size(); ++i)
            {
                C cx = (C)x[i];
                *this & cx;
            }
        }
        
        template<typename C, typename T, unsigned int N>
        void arrayFixedCast(T (&x)[N])
        {
            for (unsigned int i = 0; i < N; ++i)
            {
                C cx = (C)x[i];
                *this & cx;
            }
        }
    };
    
    class ReadBuffer
    {
    public:
        virtual ~ReadBuffer() {}
        
        virtual uint8_t readByte() = 0;
        
        void operator&(uint8_t &value) { read(value); }
        void operator&(int8_t &value) { read(value); }
        void operator&(uint16_t &value) { read(value); }
        void operator&(int16_t &value) { read(value); }
        void operator&(uint32_t &value) { read(value); }
        void operator&(int32_t &value) { read(value); }
        void operator&(uint64_t &value) { read(value); }
        void operator&(int64_t &value) { read(value); }
        void operator&(float &value) { read(value); }
        void operator&(double &value) { read(value); }
        
        void operator&(bool &value)
        {
            uint8_t x;
            read(x);
            value = (x != 0);
        }
 
        template<typename T>
        void read(T &value)
        {
            for (unsigned int i = 0; i < sizeof(T); ++i)
            {
                ((uint8_t *)&value)[i] = readByte();
            }
        }
        
        void operator&(std::string &value)
        {
            uint32_t size;
            *this & size;
            value.resize(size);
            for (unsigned int i = 0; i  < value.size(); ++i)
            {
                *this & (int8_t &)value[i];
            }
        }
        
        template<typename C, typename T>
        void memberCast(T &x)
        {
            C cx;
            *this & cx;
            x = (T)cx;
        }
        
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
    
        template<typename S, typename C, typename T>
        void arrayVariableCast(std::vector<T> &x)
        {
            S size;
            *this & size;
            x.resize(size);
            C cx;
            for (S i = 0; i < size; ++i)
            {
                *this & cx;
                x[i] = (T)cx;
            }
        }
        
        template<typename C, typename T, unsigned int N>
        void arrayFixedCast(T (&x)[N])
        {
            C cx;
            for (unsigned int i = 0; i < N; ++i)
            {
                *this & cx;
                x[i] = (T)cx;
            }
        }
    };
    
    class FileBuffer: public WriteBuffer, public ReadBuffer
    {
    public:
        FileBuffer(FILE *fp)
        {
            _fp = fp;
        }
        
        ~FileBuffer()
        {
            if (_fp)
            {
                fclose(_fp);
            }
        }
        
        virtual void writeByte(uint8_t value)
        {
            fputc(value, _fp);
        }
    
        virtual uint8_t readByte()
        {
            return fgetc(_fp);
        }
    
    protected:
        FILE *_fp;
    };

    class MemoryBuffer: public WriteBuffer, public ReadBuffer
    {
    public:
        MemoryBuffer()
        {
            _readPos = 0;
        }
        
        void rewind()
        {
            _readPos = 0;
        }
        
        virtual void writeByte(uint8_t value)
        {
            data.push_back(value);
        }
        
        virtual uint8_t readByte()
        {
            if (_readPos >= data.size())
            {
                return 0;
            } else {
                return data[_readPos++];
            }
        }
        
        std::vector<uint8_t> data;

    protected:
        unsigned int _readPos;
    };
}
