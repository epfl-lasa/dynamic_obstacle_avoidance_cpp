/**
 * @class StarShapeHull
 * @brief Class to define a star shape hull
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_STARSHAPEHULL_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_STARSHAPEHULL_H_

#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"
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
		Eigen::MatrixXd surface_points;

		Eigen::Vector3d compute_baricenter(const std::deque<std::unique_ptr<Obstacle> >& primitives);

		void compute_from_primitives(const std::deque<std::unique_ptr<Obstacle> >& primitives, double threshold=0.01, double min_radius=0.1);

	public:
		explicit StarShapeHull(const std::string& name="");

		explicit StarShapeHull(const std::deque<std::unique_ptr<Obstacle> >& primitives, unsigned int resolution=500, const std::string& name="");

		Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;

		double compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin=0.) const;

		void draw(const std::string& color="k") const;
	};
}
#endif