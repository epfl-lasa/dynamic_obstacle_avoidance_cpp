/**
 * @class Obstacle
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_OBSTACLE_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_OBSTACLE_H_

#include <eigen3/Eigen/Core>
#include "DynamicObstacleAvoidance/State/State.hpp"
#include "DynamicObstacleAvoidance/State/Pose.hpp"
#include "DynamicObstacleAvoidance/Agent.hpp"
#include <iostream>

class Obstacle 
{
private:
	State state;
	Eigen::Vector3d reference_position;

	double safety_margin;

public:
	explicit Obstacle();
	explicit Obstacle(const double& cx, const double& cy, const double& cz, const double& safety_margin=0);
	explicit Obstacle(const State& state, const double& safety_margin=0);
	explicit Obstacle(const State& state, const Eigen::Vector3d& reference_position, const double& safety_margin=0);
	~Obstacle();

	inline const State get_state() const 
	{ 
		return this->state;
	}

	inline const Pose get_pose() const 
	{ 
		return this->state.get_pose();
	}

	inline const Eigen::Vector3d get_position() const 
	{ 
		return this->state.get_position();
	}

	inline const Eigen::Quaterniond get_orientation() const 
	{ 
		return this->state.get_orientation();
	}

	inline const Eigen::Vector3d get_linear_velocity() const
	{
		return this->state.get_linear_velocity();
	}

	inline const Eigen::Vector3d get_angular_velocity() const
	{
		return this->state.get_angular_velocity();
	}

	inline const Eigen::Vector3d get_reference_position() const
	{
		return this->reference_position;
	}

	inline double get_safety_margin() const
	{ 
		return this->safety_margin;
	}

	inline void set_pose(const Pose& pose)
	{
		this->state.set_pose(pose);
	}

	inline void set_position(const Eigen::Vector3d& position)
	{
		this->state.set_position(position);
	}

	inline void set_position(const double& x, const double& y, const double& z)
	{
		this->state.set_position(x, y, z);
	}

	inline void set_orientation(const Eigen::Quaterniond& orientation)
	{
		this->state.set_orientation(orientation);
	}

	inline void set_linear_velocity(const Eigen::Vector3d& linear_velocity)
	{
		this->state.set_linear_velocity(linear_velocity);
	}

	inline void set_angular_velocity(const Eigen::Vector3d& angular_velocity)
	{
		this->state.set_angular_velocity(angular_velocity);
	}

	inline void set_reference_position(const Eigen::Vector3d& reference_position)
	{
		this->reference_position = reference_position;
	}

	virtual Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;
	
	virtual double compute_distance_to_agent(const Agent& agent) const;
};

#endif