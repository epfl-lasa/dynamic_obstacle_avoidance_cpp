/**
 * @class Obstacle
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_H_

#include <eigen3/Eigen/Core>

using namespace Eigen;

class Obstacle 
{
private:
	VectorXf position;
	VectorXf orientation;
	VectorXf axis;
	VectorXf curvature;
	double safety_margin;

public:
	explicit Obstacle(VectorXf& position, VectorXf& orientation, VectorXf& axis, VectorXf& curvature, double safety_margin);
	~Obstacle();

	inline const VectorXf get_axis() const 
	{ 
		return this->axis;
	}

	inline const VectorXf get_curvature() const 
	{ 
		return this->curvature;
	}

	inline const VectorXf get_position() const 
	{ 
		return this->position;
	}

	inline const VectorXf get_orientation() const
	{
		return this->orientation;
	}

	inline double get_safety_margin() const
	{ 
		return this->safety_margin;
	}
};

#endif