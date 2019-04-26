/**
 * @class Modulation
 * @brief Class to define Modulations of the dynamical system
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_AGENT_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_AGENT_H_

#include <eigen3/Eigen/Core>
#include "DynamicObstacleAvoidance/State/State.hpp"
#include "DynamicObstacleAvoidance/State/Pose.hpp"

class Agent
{
private:
	State state;
	double safety_margin;

public:
	explicit Agent(const State& state, const double& safety_margin=0.0);

	inline const State get_state() const 
	{ 
		return this->state;
	}

	inline const Pose get_pose() const 
	{ 
		return this->state.get_pose();
	}

	inline const Eigen::Vector3f get_position() const 
	{ 
		return this->state.get_position();
	}

	inline const Eigen::Quaternionf get_orientation() const 
	{ 
		return this->state.get_orientation();
	}

	inline const Eigen::Vector3f get_linear_velocity() const
	{
		return this->state.get_linear_velocity();
	}

	inline const Eigen::Vector3f get_angular_velocity() const
	{
		return this->state.get_angular_velocity();
	}

	inline double get_safety_margin() const
	{ 
		return this->safety_margin;
	}

	inline void set_linear_velocity(const Eigen::Vector3f& linear_velocity)
	{
		this->state.set_linear_velocity(linear_velocity);
	}
};

#endif