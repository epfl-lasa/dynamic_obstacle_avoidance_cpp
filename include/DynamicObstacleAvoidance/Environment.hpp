#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_ENVIRONMENT_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_ENVIRONMENT_H_

#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Agent.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"
#include "DynamicObstacleAvoidance/Utils/Plotting/PlottingTools.hpp"
#include "DynamicObstacleAvoidance/Utils/ObstacleGeneration/Aggregation.hpp"
#include "DynamicObstacleAvoidance/Exceptions/ObstacleNotInEnvironmentException.hpp"
#include <memory>
#include <deque>
#include <algorithm>
#include <iterator>

namespace DynamicObstacleAvoidance
{
	class Environment : public std::map<std::string, std::shared_ptr<Obstacle> >
	{
	private:
		bool aggregated;
		std::deque<std::shared_ptr<Obstacle> > obstacle_list;

	public:
		explicit Environment(bool aggregated=true);

		bool is_aggregated() const;

		void set_aggregated(bool aggregated);

		void add_obstacle(const std::shared_ptr<Obstacle>& obstacle);

		const std::deque<std::shared_ptr<Obstacle> >& get_obstacle_list() const;

		void update();

		mapped_type& operator[] (const key_type& k);
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