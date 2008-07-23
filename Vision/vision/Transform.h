#ifndef _VISION__TRANSFORM_H_
#define _VISION__TRANSFORM_H_

#include <vector>
#include <Geometry/Point2d.hpp>
#include <eigen/matrix.h>
#include <QDomElement>

namespace Vision
{
	class Distortion;
	
	class Transform
	{
	public:
		Transform();
		
		// Transforms a point
		Geometry::Point2d transform(Geometry::Point2d pt);
		
		// Calculates a transformation matrix based on image and world reference points
		bool calculate_matrix(Distortion *distortion); 
		
		void load(QDomElement element);
		void save(QDomElement element);
		
		// Reference points in raw image space
	    Geometry::Point2d image_point[3];

	    // Reference points in world space
	    Geometry::Point2d world_point[3];
		
	    // Frame to world coordinate transformation.
	    // World = transform * Frame
	    Eigen::Matrix3f matrix;

	protected:
		void load_matrix(QDomElement element, const char *name);
		void save_matrix(QDomElement element, const char *name);
	};
}

#endif // _VISION__TRANSFORM_H_
