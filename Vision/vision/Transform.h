#pragma once

#include <vector>
#include <Geometry2d/Point.hpp>
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
		Geometry2d::Point transform(Geometry2d::Point pt);
		
		// Calculates a transformation matrix based on image and world reference points
		bool calculate_matrix(Distortion *distortion); 
		
		void load(QDomElement element);
		void save(QDomElement element);
		
		// Reference points in raw image space
	    Geometry2d::Point image_point[3];

	    // Reference points in world space
	    Geometry2d::Point world_point[3];
		
	    // Frame to world coordinate transformation.
	    // World = transform * Frame
	    Eigen::Matrix3f matrix;

	protected:
		void load_matrix(QDomElement element, const char *name);
		void save_matrix(QDomElement element, const char *name);
	};
}
