#ifndef _VISION__COLORSEG_H_
#define _VISION__COLORSEG_H_

#include "Colors.h"
#include "Process.h"
#include "../Camera_Thread.h"

#include <QImage>
#include <QDomElement>

class Camera_Thread;
class Image;

namespace Vision
{
	class Colorseg: public Processor
	{
		public:
			Colorseg(Camera_Thread *camera_thread);
			~Colorseg();

			void run();
			void run(const Image *frame);

			const QImage &output() const
			{
				return _output;
			}

			uint8_t &lut(QRgb in);

			// Changes all instances of (from) in the LUT to (to).
			void change_lut(uint8_t from, uint8_t to);

			// Turns off the bit for <color> in all LUT entries.
			void clear_color(int color);

			void load(QDomElement element);
			void save(QDomElement element);

			// Used by Spanners
			std::vector<uint32_t> color_list[Num_Colors];
			std::vector<unsigned int> line_start[Num_Colors];
			int color_count[Num_Colors];

			// Pixel format is ARGB.  Blue is in the LSB.
			//
			// How many bits of the original pixel are used in the quantized pixel.
			static const int red_bits = 4;
			static const int green_bits = 4;
			static const int blue_bits = 4;

		protected:
			// Mask for the bits in the original pixel that are used for each
			// component in the quantized pixel.
			static const uint32_t red_mask = ((1 << red_bits) - 1) << (8
			        - red_bits + 16);
			static const uint32_t green_mask = ((1 << green_bits) - 1) << (8
			        - green_bits + 8);
			static const uint32_t blue_mask = ((1 << blue_bits) - 1) << (8
			        - blue_bits);

			// How far down to shift the original pixel to get the used bits
			// in the right position in the quantized pixel.
			static const int blue_shift = 8 - blue_bits;
			static const int green_shift = 8 - green_bits + blue_shift;
			static const int red_shift = 8 - red_bits + green_shift;

			static const int lut_size = 1 << (red_bits + green_bits + blue_bits);

			// Converts a pixel from an original RGB32 image to a quanztized pixel
			// which is an index in the LUT.
			static uint32_t quantize(uint32_t pixel)
			{
				return ((pixel & red_mask) >> red_shift)
				        | ((pixel & green_mask) >> green_shift) | ((pixel
				        & blue_mask) >> blue_shift);
			}

			uint8_t _lut[lut_size];

			Camera_Thread *_camera_thread;
			QImage _output;
	};
}
;

#endif // _VISION__COLORSEG_H_
