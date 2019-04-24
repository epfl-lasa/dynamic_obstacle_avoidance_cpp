/**
 * @class Obstacle
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_OBSTACLE_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_OBSTACLE_H_

#include <eigen3/Eigen/Core>

class Obstacle 
{
private:
	Eigen::Vector3f center_position;
	Eigen::Vector4f center_orientation;
	Eigen::Vector3f reference_position;

	Eigen::Vector3f linear_velocity;
	Eigen::Vector3f angular_velocity;
	double safety_margin;

public:
	explicit Obstacle(Eigen::Vector3f& center_position, Eigen::Vector4f& center_orientation, Eigen::Vector3f& reference_position, double safety_margin=0);
	explicit Obstacle(Eigen::Vector3f& center_position, Eigen::Vector4f& center_orientation, double safety_margin=0);
	~Obstacle();

	inline const Eigen::Vector3f get_center_position() const 
	{ 
		return this->center_position;
	}

	inline const Eigen::Vector4f get_center_orientation() const 
	{ 
		return this->center_orientation;
	}

	inline const Eigen::Vector3f get_reference_position() const 
	{ 
		return this->reference_position;
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

	virtual Eigen::Vector3f compute_normal_to_external_point(const Eigen::Vector3f& external_point) const;
	virtual double compute_distance_to_external_point(const Eigen::Vector3f& external_point) const;
};

#endif