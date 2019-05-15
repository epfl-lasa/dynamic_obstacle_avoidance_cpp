/**
 * @class Obstacle
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_LASERSCAN2D_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_LASERSCAN2D_H_

#include <eigen3/Eigen/Core>
#include <deque>
#include <utility>
#include <math.h>
#include <armadillo>
#include <iostream>
#include <mutex>

#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/State/State.hpp"
#include "DynamicObstacleAvoidance/State/Pose.hpp"

class PointCloudToObstacle
{
private:
	std::mutex mutex;
	
	arma::gmm_full learn_gmm_on_points(const Eigen::MatrixXd& surface_points, const int& nb_gaussians=2);

	std::pair<arma::gmm_full, double> compute_bic(const arma::mat& data, const int& nb_gaussians);

	std::deque<Eigen::MatrixXd> cluster_surface_points(const Eigen::MatrixXd& surface_points);

	Eigen::MatrixXd from_laser_scan_to_point_cloud(const Eigen::ArrayXd& distances, const double& starting_angle, const double& delta_angle, const double& z_coordinate) const;

public:
	explicit PointCloudToObstacle(const double& safety_margin=0);
	~PointCloudToObstacle();

	std::deque<Ellipsoid> fit_ellipsoids(const Eigen::MatrixXd& surface_points, const int& nb_ellipsoids);

	std::deque<Ellipsoid> fit_ellipsoids(const Eigen::ArrayXd& distances, const double& starting_angle, const double& delta_angle, const double& z_coordinate, const int& nb_ellipsoids);
};

#endif