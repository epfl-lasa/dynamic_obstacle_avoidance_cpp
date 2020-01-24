/**
 * @class StarShapeHull
 * @brief Class to define a star shape hull
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_STARSHAPEHULL_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_STARSHAPEHULL_H_

#include "DynamicObstacleAvoidance/Agent.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"
#include <algorithm>
#include <deque>
#define _USE_MATH_DEFINES 
#include <cmath>

namespace plt = matplotlibcpp;

namespace DynamicObstacleAvoidance
{
	class StarShapeHull: public Obstacle
	{
	private:
		bool is_inside;
		unsigned int resolution;
		double min_radius;
		Eigen::MatrixXd cartesian_surface_points;
		Eigen::MatrixXd polar_surface_points;

		Eigen::Vector3d compute_baricenter(const std::deque<std::shared_ptr<Obstacle> >& primitives);

	public:
		explicit StarShapeHull(bool is_inside=false, unsigned int resolution=500, double min_radius=0.5);

		explicit StarShapeHull(const std::deque<std::shared_ptr<Obstacle> >& primitives, bool is_inside=false, unsigned int resolution=1000, double min_radius=0.5);

		~StarShapeHull();

		double get_min_radius() const;

		Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;

		double compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin=0.) const;

		void draw(const std::string& color="k") const;

		void set_resolution(unsigned int resolution);

		void compute_from_primitives(const std::deque<std::shared_ptr<Obstacle> >& primitives, Eigen::Vector3d reference_point);

		void compute_from_primitives(const std::deque<std::shared_ptr<Obstacle> >& primitives);

		bool point_is_inside(const Eigen::Vector3d& point) const;

		bool is_closed() const;
	};

	inline double StarShapeHull::get_min_radius() const
	{
		return this->min_radius;
	}

	inline void StarShapeHull::set_resolution(unsigned int resolution)
	{
		this->resolution = resolution;
		this->cartesian_surface_points = Eigen::MatrixXd::Zero(3, resolution);
		this->polar_surface_points = Eigen::MatrixXd::Zero(3, resolution);
	}
}
#endif