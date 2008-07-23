#include "Image.h"

#include <stdlib.h>
#include <string.h>
#include <stdexcept>

#include <QImage>

using namespace std;

Image::Image()
{
    _width = _height = _bpp = 0;
    _data = 0;
    _allocated = false;
}

Image::Image(const Image &other)
{
    _width = _height = _bpp = 0;
    _data = 0;
    _allocated = false;
    *this = other;
}

Image::Image(const QImage &other)
{
    QImage::Format format = other.format();
    if (format != QImage::Format_RGB32 && format != QImage::Format_ARGB32 && format != QImage::Format_ARGB32_Premultiplied)
    {
        throw runtime_error("Image::Image(const QImage &) got QImage with unsupported format");
    }
    
    _width = other.width();
    _height = other.height();
    _bpp = 4;
    // You'd better not change the data.
    _data = (void *)other.bits();
    _allocated = false;
}

Image::Image(int width, int height, int bpp)
{
    _width = width;
    _height = height;
    _bpp = bpp;
    
    // Round up to a multiple of four bytes
    unsigned int n = (_width * _height * _bpp + 3) & ~4;
    _data = calloc(n, 1);
    _allocated = true;
}

Image::~Image()
{
    if (_data && _allocated)
    {
        free(_data);
    }
}

const Image &Image::operator=(const Image &other)
{
    if (_data && _allocated)
    {
        free(_data);
    }
    
    _width = other._width;
    _height = other._height;
    _bpp = other._bpp;
    
    // Copy data
    if (_width && _height && _bpp && other._data)
    {
        unsigned int n = (_width * _height * _bpp + 3) & ~4;
        _data = malloc(n);
        _allocated = true;
        memcpy(_data, other._data, bytes());
    } else {
        _data = 0;
    }
    
    return *this;
}

QRgb Image::pixel(int x, int y) const
{
    if (x < 0 || y < 0 || x >= _width || y >= _height)
    {
        return qRgb(0, 0, 0);
    }
    
    uint8_t *p = (uint8_t *)_data + x * _bpp + y * _width * _bpp;
    return qRgb(p[2], p[1], p[0]);
}
