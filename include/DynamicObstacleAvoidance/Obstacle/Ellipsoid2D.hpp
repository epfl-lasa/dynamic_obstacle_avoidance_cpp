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


class Ellipsoid2D: public Obstacle 
{
private:
	Eigen::Array2f axis_lengths;
	Eigen::Array2f curvature_factor;

public:
	explicit Ellipsoid2D(const State& state, const float& safety_margin=0);
	explicit Ellipsoid2D(const State& state, const Eigen::Vector3f& reference_position, const float& safety_margin=0);
	~Ellipsoid2D();

	inline const Eigen::Array2f get_axis_lengths() const 
	{ 
		return this->axis_lengths;
	}

	inline const Eigen::Array2f get_curvature_factor() const 
	{ 
		return this->curvature_factor;
	}

	inline void set_axis_lengths(const Eigen::Array2f& axis_lengths)
	{
		this->axis_lengths = axis_lengths;
	}

	inline void set_curvature_factor(const Eigen::Array2f& curvature_factor)
	{
		this->curvature_factor = curvature_factor;
	}

	Eigen::Vector3f compute_normal_to_external_point(const Eigen::Vector3f& external_point) const;
	float compute_distance_to_external_point(const Eigen::Vector3f& external_point) const;	
};

#endif