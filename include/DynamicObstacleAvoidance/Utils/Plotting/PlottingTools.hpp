#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_PLOTTING_PLOTTINGTOOLS_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_PLOTTING_PLOTTINGTOOLS_H_

#include <deque>
#include <vector>
#include <eigen3/Eigen/Core>
#include <memory>
#include <string>

#include "DynamicObstacleAvoidance/Agent.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Utils/Plotting/matplotlibcpp.hpp"

namespace plt = matplotlibcpp;

namespace PlottingTools 
{
	std::vector<double> linespace(const double& start, const double& ed, const int& num);

	void plot_clusters(const std::deque<Eigen::MatrixXd>& clusters, bool is_show=true);

	void plot_fitted_clusters(const std::deque<Eigen::MatrixXd>& clusters, const std::deque<std::unique_ptr<Obstacle> >& obstacles, bool is_show=true);

	void plot_configuration(const Agent& agent, const std::deque<std::unique_ptr<Obstacle> >& obstacles, const Eigen::Vector3d& goal, const std::deque<Eigen::Vector3d>& position_history, const std::string& savefile="", const bool& is_show=true);
}

#endif