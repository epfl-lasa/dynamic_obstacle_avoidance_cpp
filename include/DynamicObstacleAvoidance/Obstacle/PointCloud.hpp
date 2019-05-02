/**
 * @class Obstacle
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_POINTCLOUD_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_POINTCLOUD_H_

#include <eigen3/Eigen/Core>
#include <list>
#include "DynamicObstacleAvoidance/State/State.hpp"
#include "DynamicObstacleAvoidance/State/Pose.hpp"

class PointCloud 
{
private:
	std::list<Eigen::Vector3f> points;

	float safety_margin;

public:
	explicit PointCloud(const std::list<Eigen::Vector3f>& points, const float& safety_margin=0);
	~PointCloud();

	Eigen::Vector3f compute_normal_to_external_point(const Eigen::Vector3f& external_point) const;
	
	float compute_distance_to_external_point(const Eigen::Vector3f& external_point) const;
};

#endif