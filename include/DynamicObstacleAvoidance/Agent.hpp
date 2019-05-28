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
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"

class Agent
{
private:
	State state;
	double safety_margin;
	std::unique_ptr<Obstacle> envelope;
	double delta_t;

	void update_envelope();

public:
	Agent(const Agent& agent);

	explicit Agent(const State& state, const double& safety_margin=0.0);

	inline const Obstacle& get_envelope() const 
	{ 
		return *(this->envelope);
	}

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

	inline double get_safety_margin() const
	{ 
		return this->safety_margin;
	}

	inline void set_safety_margin(const double& safety_margin)
	{ 
		this->safety_margin = safety_margin;
	}

	inline void set_pose(const Pose& pose)
	{
		this->state.set_pose(pose);
		this->update_envelope();
	}

	inline void set_position(const Eigen::Vector3d& position)
	{
		this->state.set_position(position);
		this->update_envelope();
	}

	inline void set_orientation(const Eigen::Quaterniond& orientation)
	{
		this->state.set_orientation(orientation);
		this->update_envelope();
	}

	inline void set_linear_velocity(const Eigen::Vector3d& linear_velocity)
	{
		this->state.set_linear_velocity(linear_velocity);
		this->update_envelope();
	}

	inline void set_angular_velocity(const Eigen::Vector3d& angular_velocity)
	{
		this->state.set_angular_velocity(angular_velocity);
		this->update_envelope();
	}

	inline void transform(const Pose& p)
	{
		this->set_pose(p * this->get_pose());
	}

	inline const Agent transformed(const Pose& p) const
	{
		Agent agent(*this);
		agent.transform(p);
		return agent;
	}

	inline friend std::ostream& operator<<(std::ostream& os, const Agent& agent) 
	{ 
		os << "Agent" << std::endl;
  		os << agent.state << std::endl;
  		os << "safety margin: " << agent.safety_margin;
  		return os;
	}
};

#endif