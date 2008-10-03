#include "Image_Texture.h"
#include "Image.h"

#include <QImage>
#include <GL/gl.h>
#include <stdexcept>

using namespace std;

Image_Texture::Image_Texture()
{
    _internal_format = -1;
    _format = -1;
    _id = 0;
    _inited = false;
}

Image_Texture::Image_Texture(int internal_format, int format)
{
    _internal_format = internal_format;
    _format = format;
    
    _id = 0;
    _inited = false;
}

void Image_Texture::generate()
{
    glGenTextures(1, &_id);
}

void Image_Texture::set_format(int internal_format, int format)
{
    if (internal_format != _internal_format || format != _format)
    {
        _inited = false;
        _internal_format = internal_format;
        _format = format;
    }
}

void Image_Texture::update(const Image &image)
{
    if (image.bpp() == 3)
    {
        set_format(3, GL_BGR);
    } else if (image.bpp() == 4)
    {
        set_format(4, GL_BGRA);
    }
    
    update(image.width(), image.height(), image.data());
}

void Image_Texture::update(const QImage &image)
{
    update(image.width(), image.height(), image.bits());
}

void Image_Texture::update(int width, int height, const void *data)
{
    if (_internal_format < 0 || _format < 0)
    {
        throw runtime_error("No format - can't use an Image_Texture created with the default constructor");
    }

	
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, _id);

    if (!_inited)
    {
	    glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, _internal_format, width, height,
			 0, _format, GL_UNSIGNED_BYTE, data);
	    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	}
	else
	{
		glTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, 0, 0, width, height, _format,
			GL_UNSIGNED_BYTE, data);
	}
}

void Image_Texture::use()
{
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, _id); //GL_TEXTURE_2D
}
