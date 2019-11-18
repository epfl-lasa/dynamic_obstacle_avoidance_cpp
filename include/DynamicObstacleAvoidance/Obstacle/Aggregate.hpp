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
#include "DynamicObstacleAvoidance/Obstacle/StarShapeHull.hpp"
#include <iostream>

namespace DynamicObstacleAvoidance
{
	class Aggregate: public Obstacle 
	{
	private:
		StarShapeHull inside_hull;
		StarShapeHull outside_hull;
		std::deque<std::unique_ptr<Obstacle> > primitives;

		void update_positions();

		Aggregate* implicit_clone() const override;

	public:
		explicit Aggregate();

		explicit Aggregate(const std::deque<std::unique_ptr<Obstacle> >& primitives);

		const auto& get_primitives() const;

		void add_primitive(const std::unique_ptr<Obstacle>& primitive, bool update_hull=true);

		void add_primitive(const Obstacle& primitive, bool update_hull=true);

		Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;
		
		double compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin) const;

		void draw(const std::string& color="k") const;

		std::ostream& print(std::ostream& os) const override;

		void update_hull();
	};

	inline const auto& Aggregate::get_primitives() const
	{
		return this->primitives;
	}

	inline std::ostream& Aggregate::print(std::ostream& os) const
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
}
#endif