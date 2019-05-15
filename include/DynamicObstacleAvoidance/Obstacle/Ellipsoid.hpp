/**
 * @class Obstacle
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_ELLIPSOID_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_ELLIPSOID_H_

#include <iostream>
#include <eigen3/Eigen/Core>
#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"


class Ellipsoid: public Obstacle 
{
private:
	Eigen::Array3d axis_lengths;
	Eigen::Array3d curvature_factor;

public:
	explicit Ellipsoid();
	explicit Ellipsoid(const double& cx, const double& cy, const double& cz, const double& safety_margin=0);
	explicit Ellipsoid(State state, const double& safety_margin=0);
	explicit Ellipsoid(const State& state, const Eigen::Vector3d& reference_position, const double& safety_margin=0);
	~Ellipsoid();

	inline const Eigen::Array3d get_axis_lengths() const 
	{ 
		return this->axis_lengths;
	}

	inline const Eigen::Array3d get_curvature_factor() const 
	{ 
		return this->curvature_factor;
	}

	inline void set_axis_lengths(const Eigen::Array3d& axis_lengths)
	{
		this->axis_lengths = axis_lengths;
	}

	inline void set_curvature_factor(const Eigen::Array3d& curvature_factor)
	{
		this->curvature_factor = curvature_factor;
	}

	Eigen::Vector3d compute_normal_to_external_point(const Eigen::Vector3d& external_point) const;
	double compute_distance_to_external_point(const Eigen::Vector3d& external_point) const;	
};

#endif