#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_MATHTOOLS_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_MATHTOOLS_H_

#include <vector>

namespace MathTools 
{
	std::vector<double> linspace(const double& start, const double& ed, const int& num);
	
	template <typename T> 
	int sign(T val) {
    	return (T(0) < val) - (val < T(0));
	}
}

#endif
