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
		StarShapeHull inside_hull; //< inside hull of the aggregate
		StarShapeHull outside_hull; //< outside hull of the aggregate
		std::deque<std::shared_ptr<Obstacle> > primitives; //< list of primitive obstacles

		/**
		 * @brief Function to update the position of the center of the aggregate
		 */
		void update_center_position();

	public:
		/**
		 * @brief Empty constructor
		 */
		explicit Aggregate();

		/**
		 * @brief Constructor from a list of primitives
		 * @param primitives the primitives composing the aggregate
		 */
		explicit Aggregate(const std::deque<std::shared_ptr<Obstacle> >& primitives);

		/**
		 * @brief Setter of the reference position
		 * @param reference_position the reference position
		 */
		void set_reference_position(const Eigen::Vector3d& reference_position) override;

		/**
		 * @brief Getter of the list of primitives
		 * @return the list of primitives
		 */
		const auto& get_primitives() const;

		/**
		 * @brief Function to add a primitive obstacle to the aggegate
		 * @param primitive the primitive to add
		 * @param update_hull if true update the hulls
		 */
		void add_primitive(const std::shared_ptr<Obstacle>& primitive, bool update_hull=true);

		/**
		 * @brief Function to compute the normal on the surface to the agent wrt the reference point 
		 * @param agent the agent 
		 * @return the normal vector
		 */
		Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;

		/**
		 * @brief Function to compute the distance between a point and the surface of the obstacle wrt the reference point 
		 * @param point the external point
		 * @param safety_margin the safety margin to add
		 * @return the distance value (1 if the point is on the surface)
		 */
		double compute_distance_to_point(const Eigen::Vector3d& point, const Eigen::Array3d& safety_margin) const;

		/**
		 * @brief Function to draw an agent
		 * @param color the color to apply
		 * @param axis axis to draw (default = "xy")
		 */
		void draw(const std::string& color="k", const std::string& axis="xy") const;

		/**
		 * @brief Overload of the print function
		 * @param os the current stream
		 * @return the strem appended with the obstacle description
		 */
		std::ostream& print(std::ostream& os) const override;

		/**
		 * @brief Function to update the hulls
		 */
		void update_hull();

		/**
		 * @brief Function to check if a point is inside the obstacle
		 * @param point the point
		 * @return true if the point is inside
		 */
		bool point_is_inside(const Eigen::Vector3d& point) const;

		/**
		 * @brief Function to compute the repulsion vector when the agent is inside the obstacle
		 * @param agent the agent
		 * @return the repulsion vector
		 */
		Eigen::Vector3d compute_repulsion_vector(const Agent& agent) const;

		/**
		 * @brief Function to check if the aggregate is closed, i.e. every surface point is farther than the min radius
		 * @return true if the aggregate is closed
		 */
		bool is_closed() const;
	};

	inline void Aggregate::set_reference_position(const Eigen::Vector3d& reference_position)
	{
		this->Obstacle::set_reference_position(reference_position);
		for(auto& o:this->primitives)
		{
			o->set_reference_position(reference_position);
		}
	}

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

	inline bool Aggregate::is_closed() const
	{
		return this->outside_hull.is_closed();
	}
}
#endif