/**
 * @class Obstacle
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_ELLIPSOID_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_ELLIPSOID_H_

#include <eigen3/Eigen/Core>
#define _USE_MATH_DEFINES 
#include <cmath>
#include <vector>
#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"


class Ellipsoid: public Obstacle 
{
private:
	Eigen::Array3d axis_lengths;
	Eigen::Array3d curvature_factor;
	double epsilon;

	bool is_intersecting_ellipsoid(const Ellipsoid& other_obstacle) const;

	Ellipsoid* implicit_clone() const override;

public:
	explicit Ellipsoid();

	explicit Ellipsoid(const Ellipsoid& ellipsoid);

	explicit Ellipsoid(const double& cx, const double& cy, const double& cz, const double& safety_margin=0);

	explicit Ellipsoid(const State& state, const double& safety_margin=0);

	explicit Ellipsoid(const State& state, const Eigen::Vector3d& reference_position, const double& safety_margin=0);
	~Ellipsoid();

	inline const Eigen::Array3d get_axis_lengths() const 
	{ 
		return this->axis_lengths;
	}

	inline double get_axis_lengths(const int& index) const 
	{ 
		return this->axis_lengths(index);
	}

	inline const Eigen::Array3d get_curvature_factor() const 
	{ 
		return this->curvature_factor;
	}

	inline double get_curvature_factor(const int& index) const 
	{ 
		return this->curvature_factor(index);
	}

	inline void set_axis_lengths(const Eigen::Array3d& axis_lengths)
	{
		this->axis_lengths = axis_lengths;
	}

	inline void set_curvature_factor(const Eigen::Array3d& curvature_factor)
	{
		this->curvature_factor = curvature_factor;
	}

	Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;

	double compute_distance_to_agent(const Agent& agent) const;

	void draw() const;

	inline std::ostream& print(std::ostream& os) const override
	{ 
		os << static_cast<Obstacle>(*this) << std::endl;
		os << "axis lengths: (" << this->axis_lengths(0) << ", ";
		os << this->axis_lengths(1) << ", ";
		os << this->axis_lengths(2) << ")" << std::endl;
		os << "curvature factor: (" << this->curvature_factor(0) << ", ";
		os << this->curvature_factor(1) << ", ";
		os << this->curvature_factor(2) << ")";
  		return os;
	}
};

#endif