#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_ENVIRONMENT_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_ENVIRONMENT_H_

#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Agent.hpp"
#include <memory>
#include <deque>
#include <algorithm>
#include <iterator>

namespace DynamicObstacleAvoidance
{
	class Environment : public std::map<std::string, std::shared_ptr<Obstacle> >
	{
	public:
		explicit Environment();

		void add_obstacle(const std::shared_ptr<Obstacle>& obstacle);

		const std::deque<std::shared_ptr<Obstacle> > get_obstacle_list() const;
	};
}
#endif