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
	std::vector<double> linespace(const double& start, const double& ed, const int& num);

	void plot_ellipsoids2D(const std::deque<Ellipsoid>& ellipsoids, bool is_show=true);

	void plot_clusters(const std::deque<Eigen::MatrixXd>& clusters, bool is_show=true);

	void plot_fitted_clusters(const std::deque<Eigen::MatrixXd>& clusters, const std::deque<Ellipsoid>& ellipsoids, bool is_show=true);
}

#endif