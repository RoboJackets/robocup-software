#include "Colorseg.h"
#include "../Image.h"

#include <stdexcept>
#include <sys/time.h>
#include <boost/format.hpp>

using namespace std;
using namespace boost;

Vision::Colorseg::Colorseg()
{
    memset(_lut, 0, lut_size);
    
#if 0
    printf("red   mask %08x >> %d\n", red_mask, red_shift);
    printf("green mask %08x >> %d\n", green_mask, green_shift);
    printf("blue  mask %08x >> %d\n", blue_mask, blue_shift);
    printf("final red   %08x\n", (red_mask >> red_shift));
    printf("final green %08x\n", (green_mask >> green_shift));
    printf("final blue  %08x\n", (blue_mask >> blue_shift));
    printf("sanity %08x %08x\n", (red_mask >> red_shift) | (green_mask >> green_shift) | (blue_mask >> blue_shift), lut_size - 1);
#endif
}

Vision::Colorseg::~Colorseg()
{
}

uint8_t &Vision::Colorseg::lut(QRgb in)
{
    uint32_t in_value = *(const uint32_t *)&in;
    return _lut[quantize(in_value)];
}

void Vision::Colorseg::clear_color(int color)
{
    for (int i = 0; i < lut_size; ++i)
    {
        _lut[i] &= ~(1 << color);
    }
}

void Vision::Colorseg::load(QDomElement element)
{
	QString text = element.text();
	static const QString skip = "\r\n\t ";
	int pos = 0;
	for (int i = 0; i < lut_size; ++i)
	{
		QString byte_str(2, ' ');
		int start_pos = pos;
		
		// Read two characters, skipping whitespace
		for (int j = 0; j < 2; ++j)
		{
			do
			{
				if (pos >= text.size())
				{
					throw runtime_error(str(format("Colorseg::load: Text too short: only have %d bytes in %d characters")
							% i
							% text.size()));
				}
				
				byte_str[j] = text[pos++];
			} while (skip.contains(byte_str[j]));
		}
		
		// Interpret as a 2-digit hex number and store in the LUT
		bool ok = false;
		_lut[i] = byte_str.toInt(&ok, 16);
		if (!ok)
		{
			throw runtime_error(str(format("Bad byte text \"%s\" at position %d")
					% byte_str.toAscii().constData()
					% start_pos));
		}
	}
}

void Vision::Colorseg::save(QDomElement element)
{
	static const int Bytes_Per_Line = 25;

	QString text = "\n";
	for (int i = 0; i < lut_size; ++i)
	{
		QString byte_str;
		text += byte_str.sprintf("%02x ", _lut[i]);
		
		if ((i % Bytes_Per_Line) == (Bytes_Per_Line - 1))
		{
			text += '\n';
		}
	}
	
	element.appendChild(element.ownerDocument().createTextNode(text));
}

void Vision::Colorseg::run()
{
	//TODO fixme
    //run(_camera_thread->frame());
}

void Vision::Colorseg::run(const Image *frame)
{
    int w = frame->width(), h = frame->height();
    int num_pixels = w * h;
    
    if (_output.isNull() || _output.width() != w || _output.height() != h)
    {
        _output = QImage(w, h, QImage::Format_Indexed8);
        
        for (int i = 0; i < Num_Colors; ++i)
        {
            color_list[i].resize(num_pixels + 1);
            line_start[i].resize(h + 1);
        }
    }
    
    for (int i = 0; i < Num_Colors; ++i)
    {
        color_count[i] = 0;
    }
    
    const uint32_t *in = (const uint32_t *)frame->data();
    int bpp = frame->bpp();
    uint8_t *out = _output.bits();
    int i = 0;
    for (int y = 0; y < h; ++y)
    {
        for (int j = 0; j < Num_Colors; ++j)
        {
            line_start[j][y] = color_count[j];
        }
        
        for (int x = 0; x < w; ++x, ++i)
        {
            uint8_t seg = _lut[quantize(*in)];
            in = (const uint32_t *)((uint8_t *)in + bpp);
            *out++ = seg;
    
            for (int j = 0; seg && j < Num_Colors; ++j)
            {
                if (seg & 1)
                {
                    color_list[j][color_count[j]++] = i;
                }
                seg >>= 1;
            }
        }
    }
    
    // Add a sentinel for each color
    for (int i = 0; i < Num_Colors; ++i)
    {
        line_start[i][h] = color_count[i];
    }
}
