/**
 * @class State
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_STATE_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_STATE_H_

#include <eigen3/Eigen/Core>
#include <iostream>
#include "DynamicObstacleAvoidance/State/Pose.hpp"

class State
{
private:
	Pose pose;
	Eigen::Vector3d linear_velocity;
	Eigen::Vector3d angular_velocity;

public:
	explicit State(const Pose& pose);
	explicit State(const double& x, const double& y, const double& z);
	explicit State(const Eigen::Vector3d& position);
	explicit State(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
	explicit State(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity);

	inline const Pose get_pose() const 
	{ 
		return this->pose;
	}

	inline const Eigen::Vector3d get_position() const 
	{ 
		return this->pose.get_position();
	}

	inline const Eigen::Quaterniond get_orientation() const 
	{ 
		return this->pose.get_orientation();
	}

	inline const Eigen::Vector3d get_linear_velocity() const
	{
		return this->linear_velocity;
	}

	inline const Eigen::Vector3d get_angular_velocity() const
	{
		return this->angular_velocity;
	}

	inline void set_pose(const Pose& pose)
	{
		this->pose = pose;
	}

	inline void set_position(const Eigen::Vector3d& position)
	{
		this->pose.set_position(position);
	}

	inline void set_position(const double& x, const double& y, const double& z)
	{
		this->pose.set_position(x, y, z);
	}

	inline void set_orientation(const Eigen::Quaterniond& orientation)
	{
		this->pose.set_orientation(orientation);
	}

	inline void set_linear_velocity(const Eigen::Vector3d& linear_velocity)
	{
		this->linear_velocity = linear_velocity;
	}

	inline void set_angular_velocity(const Eigen::Vector3d& angular_velocity)
	{
		this->angular_velocity = angular_velocity;
	}

	inline friend std::ostream& operator<<(std::ostream& os, const State& state) 
	{ 
  		os << state.pose << std::endl;
  		os << "linear velocity: (" << state.linear_velocity(0) << ", ";
  		os << state.linear_velocity(1) << ", ";
  		os << state.linear_velocity(2) << ")" << std::endl;
  		os << "angular velocity: (" << state.angular_velocity(0) << ", ";
  		os << state.angular_velocity(1) << ", ";
  		os << state.angular_velocity(2) << ")";
  		return os;
	}
};

#endif