/**
 * @class Pose
 * @brief Class to define the obstacles
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_STATE_POSE_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_STATE_POSE_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class Pose
{
private:
	Eigen::Vector3f position;
	Eigen::Quaternionf orientation;

public:
	explicit Pose();
	explicit Pose(const Eigen::Vector3f& position);
	explicit Pose(const Eigen::Vector3f& position, const Eigen::Quaternionf& orientation);
	explicit Pose(const float& x, const float& y, const float& z);
	explicit Pose(const float& px, const float& py, const float& pz, const float& qx, const float& qy, const float& qz, const float& qw);
	Pose(const Pose& p);

	inline const Eigen::Vector3f get_position() const 
	{ 
		return this->position;
	}

	inline const Eigen::Quaternionf get_orientation() const 
	{ 
		return this->orientation;
	}

	inline void set_position(const Eigen::Vector3f& position)
	{
		this->position = position;
	}

	inline void set_orientation(const Eigen::Quaternionf& orientation)
	{
		this->orientation = orientation;
	}

	inline Pose& operator*=(const Pose& p)
	{
		this->position += this->orientation * p.position;
		this->orientation *= p.orientation;
		return (*this);
	}

	inline const Pose operator*(const Pose& p) const
	{
		Pose result(*this);
		result *= p;
		return result;
	}

	inline const Eigen::Vector3f operator*(const Eigen::Vector3f& v) const
	{
		return this->orientation * v + this->position;
	}

	inline const Pose inverse() const
	{
		Pose result(*this);
		result.orientation = this->orientation.conjugate();
		result.position = result.orientation * (- this->position);
		return result;
	}
};

#endif