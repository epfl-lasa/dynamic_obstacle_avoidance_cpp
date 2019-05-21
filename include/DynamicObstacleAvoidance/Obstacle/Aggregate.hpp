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

#include <iostream>


class Aggregate: public Obstacle 
{
private:
	std::deque<std::unique_ptr<Obstacle> > primitives;

public:
	explicit Aggregate(std::deque<std::unique_ptr<Obstacle> >& primitives);

	Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;
	
	double compute_distance_to_agent(const Agent& agent) const;

	void draw() const;
};
#endif