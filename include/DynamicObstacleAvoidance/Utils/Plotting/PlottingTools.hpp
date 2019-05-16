#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_PLOTTING_PLOTTINGSTOOLS_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_PLOTTING_PLOTTINGSTOOLS_H_

#include <deque>
#include <vector>
#define _USE_MATH_DEFINES 
#include <cmath>
#include <eigen3/Eigen/Core>

#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Utils/Plotting/matplotlibcpp.hpp"

namespace plt = matplotlibcpp;

namespace PlottingTools 
{
	std::vector<double> linespace(double start, double ed, int num);

	void plot_ellipsoids2D(std::deque<Ellipsoid> ellipsoids);
}

#endif