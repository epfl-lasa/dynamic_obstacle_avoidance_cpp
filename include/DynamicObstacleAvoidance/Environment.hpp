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

namespace DynamicObstacleAvoidance
{
	class Environment : public std::map<std::string, std::shared_ptr<Obstacle> >
	{
	private:
		bool aggregated;

	public:
		explicit Environment(bool aggregated=true);

		bool is_aggregated() const;

		void set_aggregated(bool aggregated);

		void add_obstacle(const std::shared_ptr<Obstacle>& obstacle);

		const std::deque<std::shared_ptr<Obstacle> > get_obstacle_list() const;
	};

	inline bool Environment::is_aggregated() const
	{
		return aggregated;
	}

	inline void Environment::set_aggregated(bool aggregated)
	{
		this->aggregated = aggregated;
	}
}
#endif