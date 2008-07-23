#ifndef _IMAGE_TEXTURE_H_
#define _IMAGE_TEXTURE_H_

class QImage;
class Image;

class Image_Texture
{
public:
    Image_Texture();
    Image_Texture(int internal_format, int format);
    
    void generate();
    void set_format(int internal_format, int format);
    void update(const Image &image);
    void update(const QImage &image);
    void update(int width, int height, const void *data);
    void use();

protected:
    int _internal_format;
    int _format;
    unsigned int _id;
    bool _inited;
};

#endif // _IMAGE_TEXTURE_H_
