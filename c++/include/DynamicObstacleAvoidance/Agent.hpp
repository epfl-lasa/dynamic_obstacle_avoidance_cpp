/**
 * @class Modulation
 * @brief Class to define Modulations of the dynamical system
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_AGENT_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_AGENT_H_

#include <eigen3/Eigen/Core>

class Agent
{
private:
	Eigen::Vector3f position;
	Eigen::Vector4f orientation;
	Eigen::Vector3f linear_velocity;
	Eigen::Vector3f angular_velocity;
	double safety_margin;

public:
	explicit Agent(Eigen::Vector3f& position, Eigen::Vector4f& orientation, double safety_margin=0.0);

	inline const Eigen::Vector3f get_position() const 
	{ 
		return this->position;
	}

	inline const Eigen::Vector4f get_orientation() const 
	{ 
		return this->orientation;
	}

	inline const Eigen::Vector3f get_linear_velocity() const
	{
		return this->linear_velocity;
	}

	inline const Eigen::Vector3f get_angular_velocity() const
	{
		return this->angular_velocity;
	}

	inline double get_safety_margin() const
	{ 
		return this->safety_margin;
	}

};

#endif