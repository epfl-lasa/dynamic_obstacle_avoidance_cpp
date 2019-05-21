/**
 * @class Obstacle
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_POINTCLOUDTOOBSTACLE_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_POINTCLOUDTOOBSTACLE_H_

#include <eigen3/Eigen/Core>
#include <vector>
#include <deque>
#include <utility>
#include <math.h>
#include <iostream>
#include <mutex>
#include <memory>
#include <mlpack/methods/dbscan/dbscan.hpp>

#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/State/State.hpp"
#include "DynamicObstacleAvoidance/State/Pose.hpp"

class PointCloudToObstacle
{
private:
	std::mutex mutex;
	mlpack::dbscan::DBSCAN<> cluster_algorithm;
	double safety_margin;
	
	arma::gmm_full learn_gmm_on_points(const Eigen::MatrixXd& surface_points, const int& nb_gaussians=2);

	Eigen::MatrixXd from_laser_scan_to_point_cloud(const Eigen::ArrayXd& distances, const double& starting_angle, const double& delta_angle, const double& z_coordinate) const;

public:
	explicit PointCloudToObstacle(const double& epsilon, const int& min_points_by_cluster, const double& safety_margin=0);
	~PointCloudToObstacle();

	std::deque<Eigen::MatrixXd> cluster_surface_points(const Eigen::MatrixXd& surface_points);

	std::deque<std::unique_ptr<Ellipsoid> > fit_ellipsoids_on_cluster(const Eigen::MatrixXd& surface_points, const int& nb_ellipsoids);

	std::deque<std::unique_ptr<Obstacle> > fit_obstacles(const Eigen::MatrixXd& surface_points);
};

#endif