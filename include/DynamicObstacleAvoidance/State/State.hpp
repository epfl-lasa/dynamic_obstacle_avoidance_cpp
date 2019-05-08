/**
 * @class State
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_STATE_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_STATE_H_

#include <eigen3/Eigen/Core>
#include "DynamicObstacleAvoidance/State/Pose.hpp"

class State
{
private:
	Pose pose;
	Eigen::Vector3f linear_velocity;
	Eigen::Vector3f angular_velocity;

public:
	explicit State(const Pose& pose);
	explicit State(const Eigen::Vector3f& position);
	explicit State(const Eigen::Vector3f& position, const Eigen::Quaternionf& orientation);
	explicit State(const Eigen::Vector3f& position, const Eigen::Quaternionf& orientation, const Eigen::Vector3f& linear_velocity, const Eigen::Vector3f& angular_velocity);

	inline const Pose get_pose() const 
	{ 
		return this->pose;
	}

	inline const Eigen::Vector3f get_position() const 
	{ 
		return this->pose.get_position();
	}

	inline const Eigen::Quaternionf get_orientation() const 
	{ 
		return this->pose.get_orientation();
	}

	inline const Eigen::Vector3f get_linear_velocity() const
	{
		return this->linear_velocity;
	}

	inline const Eigen::Vector3f get_angular_velocity() const
	{
		return this->angular_velocity;
	}

	inline void set_linear_velocity(const Eigen::Vector3f& linear_velocity)
	{
		this->linear_velocity = linear_velocity;
	}
};

#endif