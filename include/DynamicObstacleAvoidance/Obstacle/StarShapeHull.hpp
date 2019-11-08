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
		unsigned int resolution; 
		Eigen::MatrixXd cartesian_surface_points;
		Eigen::MatrixXd polar_surface_points;

		Eigen::Vector3d compute_baricenter(const std::deque<std::unique_ptr<Obstacle> >& primitives);

	public:
		explicit StarShapeHull(unsigned int resolution=400, const std::string& name="");

		explicit StarShapeHull(const std::deque<std::unique_ptr<Obstacle> >& primitives, unsigned int resolution=400, const std::string& name="");

		Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;

		double compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin=0.) const;

		void draw(const std::string& color="k") const;

		void set_resolution(unsigned int resolution);

		void compute_from_primitives(const std::deque<std::unique_ptr<Obstacle> >& primitives, Eigen::Vector3d reference_point, double threshold=0.05, double min_radius=0.1, unsigned int window_size=10);
		
		void compute_from_primitives(const std::deque<std::unique_ptr<Obstacle> >& primitives, double threshold=0.05, double min_radius=0.1, unsigned int window_size=10);
	};

	inline void StarShapeHull::set_resolution(unsigned int resolution)
	{
		this->resolution = resolution;
		this->cartesian_surface_points = Eigen::MatrixXd::Zero(3, resolution);
		this->polar_surface_points = Eigen::MatrixXd::Zero(3, resolution);
	}
}
#endif