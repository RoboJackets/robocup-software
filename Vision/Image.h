#ifndef _IMAGE_H_
#define _IMAGE_H_

#include <QColor>

class QImage;

class Image
{
	public:
		Image();
		Image(const Image &other);

		// Creates an Image from a QImage.
		// This copies the bits() pointer.  Don't change data and
		// don't delete the QImage before deleting this Image.
		Image(const QImage &other);

		// bpp is *bytes* per pixel
		Image(int width, int height, int bpp);

		~Image();

		const Image &operator=(const Image &other);

		int width() const
		{
			return _width;
		}
		int height() const
		{
			return _height;
		}
		int bpp() const
		{
			return _bpp;
		}
		void* data() const
		{
			return _data;
		}
		unsigned int bytes() const
		{
			return _width * _height * _bpp;
		}
		QRgb pixel(int x, int y) const;

	protected:
		int _width, _height, _bpp;
		void* _data;
		bool _allocated;
};

#endif // _IMAGE_H_
