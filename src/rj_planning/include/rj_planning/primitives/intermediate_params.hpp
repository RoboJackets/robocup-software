#pragma once

namespace planning {

struct IntermediateParams {
	IntermediateParams()
		: min_scale(0.5),
		  max_scale(1.5),
		  min_angle(20),
		  max_angle(140),
		  num_intermediates(5),
		  step_size(0.1) {}

	double min_scale;
	double max_scale;
	double min_angle;
	double max_angle;
	int num_intermediates;
	double step_size;
};

}  // namespace planning
