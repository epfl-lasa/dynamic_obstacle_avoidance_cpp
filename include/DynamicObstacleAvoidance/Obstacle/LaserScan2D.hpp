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
#include <math.h>
#include <armadillo>

#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/State/State.hpp"
#include "DynamicObstacleAvoidance/State/Pose.hpp"

class LaserScan2D: public Obstacle
{
private:
	arma::gmm_full gmm_model;

	bool learn_gmm_on_points(const Eigen::MatrixXd& surface_points, const int& nb_gaussians=2);

public:
	explicit LaserScan2D(const Eigen::ArrayXf& distances, const float& starting_angle, const float& detla_angle, const float& safety_margin=0, const float& z_coordinate=0);
	~LaserScan2D();

	Eigen::Vector3f compute_normal_to_external_point(const Eigen::Vector3f& external_point) const;
	
	float compute_distance_to_external_point(const Eigen::Vector3f& external_point) const;
};

#endif