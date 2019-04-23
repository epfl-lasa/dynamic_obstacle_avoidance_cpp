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
	Eigen::VectorXf position;
	Eigen::VectorXf orientation;
	Eigen::VectorXf velocity;
	double safety_margin;

public:
	explicit Agent(Eigen::VectorXf& position, Eigen::VectorXf& orientation, double safety_margin=0.0);

	inline const Eigen::VectorXf get_position() const 
	{ 
		return this->position;
	}

	inline const Eigen::VectorXf get_orientation() const 
	{ 
		return this->orientation;
	}

	inline const Eigen::VectorXf get_velocity() const
	{
		return this->velocity;
	}

	inline double get_safety_margin() const
	{ 
		return this->safety_margin;
	}

};

#endif