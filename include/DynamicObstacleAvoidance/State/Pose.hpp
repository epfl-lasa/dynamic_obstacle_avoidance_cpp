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
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;

public:
	explicit Pose();
	explicit Pose(const Eigen::Vector3d& position);
	explicit Pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
	explicit Pose(const double& x, const double& y, const double& z);
	explicit Pose(const double& px, const double& py, const double& pz, const double& qx, const double& qy, const double& qz, const double& qw);
	Pose(const Pose& p);

	inline const Eigen::Vector3d get_position() const 
	{ 
		return this->position;
	}

	inline const Eigen::Quaterniond get_orientation() const 
	{ 
		return this->orientation;
	}

	inline void set_position(const Eigen::Vector3d& position)
	{
		this->position = position;
	}

	inline void set_position(const double& x, const double& y, const double& z)
	{
		this->position = Eigen::Vector3d(x, y, z);
	}

	inline void set_orientation(const Eigen::Quaterniond& orientation)
	{
		this->orientation = orientation.normalized();
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

	inline const Eigen::Vector3d operator*(const Eigen::Vector3d& v) const
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