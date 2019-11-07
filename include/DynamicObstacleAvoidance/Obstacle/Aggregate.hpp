/**
 * @class Aggregate
 * @brief Class to define an aggregation of obstacles
 * @author Baptiste Busch
 * @date 2019/05/21
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_AGGREGATE_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_AGGREGATE_H_

#include <eigen3/Eigen/Core>
#include <memory>
#include <deque>
#include <limits>
#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include <iostream>
#include <dlib/optimization.h>
#include <dlib/global_optimization.h>

using namespace std::placeholders;

namespace DynamicObstacleAvoidance
{
	class Aggregate: public Obstacle 
	{
	private:
		unsigned int nb_samples;
		double intersec_factor;
		double inside_factor;
		double distance_factor;
		double area_factor;

		std::deque<std::unique_ptr<Obstacle> > primitives;

		void update_positions();

		Aggregate* implicit_clone() const override;

		double cost_function(const dlib::matrix<double,0,1>& x);

		double cost_star_shape_hull(const Ellipsoid& e1, const Ellipsoid& e2);

	public:
		explicit Aggregate();

		explicit Aggregate(const std::deque<std::unique_ptr<Obstacle> >& primitives);

		inline const auto& get_primitives() const
		{
			return this->primitives;
		}

		void add_primitive(const std::unique_ptr<Obstacle>& primitive);

		void add_primitive(const Obstacle& primitive);

		const Obstacle& get_active_obstacle(const Agent& agent) const;

		Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;
		
		double compute_distance_to_agent(const Agent& agent) const;

		void draw(const std::string& color="k") const;

		std::pair<Ellipsoid, Ellipsoid> compute_star_shape_hull();

		inline std::ostream& print(std::ostream& os) const override
		{ 
			os << static_cast<Obstacle>(*this) << std::endl;
			os << "Composed of:" << std::endl;
			for(auto& p:this->primitives)
			{
				os << "---" << std::endl;
				os << *p << std::endl;
			}
	  		return os;
		}
	};
}
#endif