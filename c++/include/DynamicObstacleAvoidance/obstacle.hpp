/**
 * @class Obstacle
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_H_

#include <eigen3/Eigen/Core>

class Obstacle 
{
private:
	Eigen::VectorXf center_position;
	Eigen::VectorXf center_orientation;
	Eigen::VectorXf reference_position;
	double safety_margin;

public:
	explicit Obstacle(Eigen::VectorXf& center_position, Eigen::VectorXf& center_orientation, Eigen::VectorXf& reference_position, double safety_margin=0);
	explicit Obstacle(Eigen::VectorXf& center_position, Eigen::VectorXf& center_orientation, double safety_margin=0);
	~Obstacle();

	inline const Eigen::VectorXf get_center_position() const 
	{ 
		return this->center_position;
	}

	inline const Eigen::VectorXf get_center_orientation() const 
	{ 
		return this->center_orientation;
	}

	inline const Eigen::VectorXf get_reference_position() const 
	{ 
		return this->reference_position;
	}

	inline double get_safety_margin() const
	{ 
		return this->safety_margin;
	}

	Eigen::VectorXf compute_normal_to_external_point(const Eigen::VectorXf& external_point) const;
	double compute_distance_to_external_point(const Eigen::VectorXf& external_point) const;
};

#endif