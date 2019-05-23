#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_AGGREGATION_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_UTILS_AGGREGATION_H_

#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"

namespace Aggregation
{
	std::deque<std::unique_ptr<Obstacle> > aggregate_obstacles(const std::deque<std::unique_ptr<Obstacle> >& obstacles);
}
#endif