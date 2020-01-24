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

namespace DynamicObstacleAvoidance
{
	class Agent;

	class Ellipsoid: public Obstacle 
	{
	private:
		Eigen::Array3d axis_lengths;
		Eigen::Array3d curvature_factor;
		double epsilon;

		bool is_intersecting_ellipsoid(const Ellipsoid& other_obstacle) const;

		Ellipsoid* implicit_clone() const override;

	public:
		explicit Ellipsoid(const std::string& name="ellipsoid");

		explicit Ellipsoid(const Ellipsoid& ellipsoid);

		explicit Ellipsoid(double cx, double cy, double cz, double safety_margin=0, const std::string& name="ellipsoid");

		explicit Ellipsoid(const State& state, double safety_margin=0, const std::string& name="ellipsoid");

		explicit Ellipsoid(const State& state, const Eigen::Vector3d& reference_position, double safety_margin=0, const std::string& name="ellipsoid");
		
		~Ellipsoid();

		const Eigen::Array3d& get_axis_lengths() const;

		double get_axis_lengths(unsigned int index) const; 

		const Eigen::Array3d& get_curvature_factor() const; 

		double get_curvature_factor(unsigned int index) const ;

		void set_axis_lengths(const Eigen::Array3d& axis_lengths);

		void set_curvature_factor(const Eigen::Array3d& curvature_factor);

		Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;

		double compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin=0.) const;

		void draw(const std::string& color="k", bool is3D=false) const;

		std::ostream& print(std::ostream& os) const override;

		double area(bool is_include_safety_margin=true) const;

		Eigen::MatrixXd sample_from_parameterization(unsigned int nb_samples, bool is_include_safety_margin) const;

		bool point_is_inside(const Eigen::Vector3d& point) const;

		Eigen::Vector3d compute_repulsion_vector(const Agent& agent) const;
	};

	inline const Eigen::Array3d& Ellipsoid::get_axis_lengths() const 
	{ 
		return this->axis_lengths;
	}

	inline double Ellipsoid::get_axis_lengths(unsigned int index) const 
	{ 
		return this->axis_lengths(index);
	}

	inline const Eigen::Array3d& Ellipsoid::get_curvature_factor() const 
	{ 
		return this->curvature_factor;
	}

	inline double Ellipsoid::get_curvature_factor(unsigned int index) const 
	{ 
		return this->curvature_factor(index);
	}

	inline void Ellipsoid::set_axis_lengths(const Eigen::Array3d& axis_lengths)
	{
		this->axis_lengths = axis_lengths;
	}

	inline void Ellipsoid::set_curvature_factor(const Eigen::Array3d& curvature_factor)
	{
		this->curvature_factor = curvature_factor;
	}

	inline std::ostream& Ellipsoid::print(std::ostream& os) const
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
}
#endif