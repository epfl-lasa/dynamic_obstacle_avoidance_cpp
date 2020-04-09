#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_ENVIRONMENT_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_ENVIRONMENT_H_

#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Agent.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"
#include "DynamicObstacleAvoidance/Utils/Plotting/PlottingTools.hpp"
#include "DynamicObstacleAvoidance/Utils/ObstacleGeneration/Aggregation.hpp"
#include <memory>
#include <deque>
#include <algorithm>
#include <iterator>
#include <regex>

namespace DynamicObstacleAvoidance
{
	class Environment
	{
	private:
		std::map<std::string, std::shared_ptr<Obstacle> > obstacle_map; ///< map of the obstacles
		bool aggregated; //< if true, the environment creates aggregates of obstacle at update
		std::deque<std::shared_ptr<Obstacle> > obstacle_list; //< the list of obstacles, eventually aggragegated

	public:
		/**
		 * @brief Constructor of an empty environment
		 * @param aggregated if true the environment creates aggregates of obstacle at update 
		 */
		explicit Environment(bool aggregated=true);

		/**
		 * @brief Getter of the aggregated attribute
		 * @return true if aggregated
		 */
		bool is_aggregated() const;

		/**
		 * @brief Setter of the aggregated attribute
		 * @param aggregated the new aggregated value
		 */
		void set_aggregated(bool aggregated);

		/**
		 * @brief Add an obstacle to the environment
		 * @param obstacle the obstacle to add as a shared_ptr
		 */
		void add_obstacle(const std::shared_ptr<Obstacle>& obstacle);

		/**
		 * @brief Remove an obstacle from the environment without deleting the pointer
		 * @param obstacle_name the name of the obstacle to remove
		 */
		void remove_obstacle(const std::string& obstacle_name);

		/**
		 * @brief Getter of the obstacle list
		 * @return the list of obstacles
		 */
		const std::deque<std::shared_ptr<Obstacle> >& get_obstacle_list() const;

		/**
		 * @brief Update the environment, i.e. recompute the list of obstacles and eventual aggregations
		 */
		void update();

		/**
		 * @brief Add the [] operator to get the pointer to an obstacle by its name
		 * @param k the key to look for
		 * @return the pointer to the obstacle
		 */
		std::shared_ptr<Obstacle>& operator[] (const std::string& k);
	};

	inline bool Environment::is_aggregated() const
	{
		return this->aggregated;
	}

	inline void Environment::set_aggregated(bool aggregated)
	{
		this->aggregated = aggregated;
	}

	inline const std::deque<std::shared_ptr<Obstacle> >& Environment::get_obstacle_list() const
	{
		return this->obstacle_list;
	}
}
#endif