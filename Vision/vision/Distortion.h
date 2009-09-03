#pragma once

#include <Geometry2d/Point.hpp>
#include <QDomElement>
#include <QMutex>
#include <vector>

namespace Vision
{
	class Calibration_Line
	{
		public:
			std::vector<Geometry2d::Point> points;
	};


	// Lens distortion parameters
	// See "Applying and removing lens distortion in post production", Vass and Perlaki
	class Distortion
	{
		public:
			Distortion();

			Geometry2d::Point undistort(Geometry2d::Point pt) const;

			void load(QDomElement element);
			void save(QDomElement element);

			// Automatically finds k1 and k2 by calibration lines
			void auto_calibrate();

			// Calculates r from frame dimensions
			void frame_size(float w, float h);

			std::vector<Calibration_Line *> cal_lines;

		protected:
			mutable QMutex mutex;

			// Primary distortion
			float _k1;

			// Secondary distortion
			float _k2;

			// Squeeze
			float _s;

			// Asymmetric distortion
			float _lx, _ly;

			// Center of the lens in distorted pixel coordinates
			float _cx, _cy;

			// Denominator of conversion to dimensionless pixel coordinates
			float _r;

			// Minimum and maximum steps for auto calibration
			float _min_step, _max_step;

			// Auto-calibrates a single parameter
			float auto_calibrate(float &param);

			// Calculates the total error on calibration lines using the given distortion
			float auto_cal_error();
	};
}
