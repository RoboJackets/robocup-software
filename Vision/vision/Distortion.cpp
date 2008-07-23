#include "Distortion.h"
#include "../Config_File.h"

#include <assert.h>
#include <Geometry/Segment.hpp>
#include <boost/foreach.hpp>

using namespace std;

Vision::Distortion::Distortion(): mutex(QMutex::Recursive)
{
	_min_step = 0.001;
	_max_step = 10;
    
	_k1 = 0;
	_k2 = 0;
	_s = 1;
	_lx = 0;
	_ly = 0;
	_cx = 0;
	_cy = 0;
	_r = 1;
}

Geometry::Point2d Vision::Distortion::undistort(Geometry::Point2d pt) const
{
	QMutexLocker ml(&mutex);
	
	Geometry::Point2d pp, out;
	
	// Convert to dimensionless coordinates
	pp.x = (pt.x - _cx) / _r;
	pp.y = (pt.y - _cy) / _r;
	
	// Undistort
	float x2 = pp.x * pp.x;
	float y2 = pp.y * pp.y;
	
	out.x = pp.x * (1 + _k1 * x2 + _k1 * (1 + _lx) * y2 + _k2 * (x2 + y2) * (x2 + y2));
	out.y = pp.y * (1 + _k1 / _s * x2 + _k1 / _s * (1 + _ly) * y2 + _k2 / _s * (x2 + y2) * (x2 + y2));
	
	return out;
}

void Vision::Distortion::frame_size(float w, float h)
{
	QMutexLocker ml(&mutex);
	
	float w2 = w / 2;
	float h2 = h / 2;
	
	//TODO: It may be desirable to change these.
	//	Probably should add an offset in normalized coords later if you want this.
	_cx = w2;
	_cy = h2;
	
	_r = sqrtf(w2 * w2 + h2 * h2);
}

void Vision::Distortion::load(QDomElement element)
{
	QMutexLocker ml(&mutex);
	
	read_param(element, "k1", _k1);
	read_param(element, "k2", _k2);
	read_param(element, "s", _s);
	read_param(element, "lx", _lx);
	read_param(element, "ly", _ly);
}

void Vision::Distortion::save(QDomElement element)
{
	QMutexLocker ml(&mutex);
	
	element.setAttribute("k1", _k1);
	element.setAttribute("k2", _k2);
	element.setAttribute("s", _s);
	element.setAttribute("lx", _lx);
	element.setAttribute("ly", _ly);
}

void Vision::Distortion::auto_calibrate()
{
	QMutexLocker ml(&mutex);
	
	// Reset to defaults
	_k1 = 0;
	_k2 = 0;
	_s = 1;
	_lx = 0;
	_ly = 0;
	
	for (int i = 0; i < 3; ++i)
	{
		auto_calibrate(_k1);
		auto_calibrate(_k2);
	}
}

float Vision::Distortion::auto_calibrate(float &param)
{
	float step = _max_step;
	float value = param;
	float best_error = 0;
	while (step > _min_step)
	{
		param = value;
		float error_stay = auto_cal_error();
		
		param = value - step;
		float error_down = auto_cal_error();
		
		param = value + step;
		float error_up = auto_cal_error();
		
#if 1
		assert(__fpclassify(error_stay) == FP_NORMAL);
		assert(__fpclassify(error_up) == FP_NORMAL);
		assert(__fpclassify(error_down) == FP_NORMAL);
#endif
		
		if (error_stay < error_down && error_stay < error_up)
		{
			step /= 4;
			best_error = error_stay;
		} else if (error_down < error_up)
		{
			value -= step;
			best_error = error_down;
		} else {
			value += step;
			best_error = error_up;
		}
		
		printf("auto cal step %f value %f best %f\n", step, value, best_error);
	}
	
	return best_error;
}

float Vision::Distortion::auto_cal_error()
{
	float error = 0;
	int n = 0;
	BOOST_FOREACH(const Calibration_Line *line, cal_lines)
	{
		Geometry::Segment seg(undistort(line->points.front()),
				undistort(line->points.back()));
		
		for (unsigned int i = 1; i < line->points.size() - 1; ++i)
		{
			error += seg.distTo(undistort(line->points[i]));
			++n;
		}
	}
	
	return error / (float)n;
}
