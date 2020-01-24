#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_AGGREGATION_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_AGGREGATION_H_

#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"

namespace DynamicObstacleAvoidance
{
	namespace Aggregation
	{
		std::deque<std::shared_ptr<Obstacle> > aggregate_obstacles(const std::deque<std::shared_ptr<Obstacle> >& obstacles);
	}
}
#endif