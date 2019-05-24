#include "DynamicObstacleAvoidance/State/Pose.hpp"

Pose::Pose():position(0,0,0), orientation(1,0,0,0)
{}

Pose::Pose(const Eigen::Vector3d& position):position(position), orientation(1,0,0,0)
{}

Pose::Pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation):
position(position), orientation(orientation)
{
	if(abs(this->orientation.norm()) - 1 < 1e-4) this->orientation.normalize();
	if(this->orientation.w() < 0.0) this->orientation = Eigen::Quaterniond(-this->orientation.w(), -this->orientation.x(), -this->orientation.y(), -this->orientation.z()); 
}

Pose::Pose(const double& x, const double& y, const double& z):
position(x,y,z), orientation(1,0,0,0)
{}

Pose::Pose(const double& px, const double& py, const double& pz, const double& qw, const double& qx, const double& qy, const double& qz):
position(px,py,pz), orientation(qw,qx,qy,qz)
{
	if(abs(orientation.norm()) - 1 < 1e-4) this->orientation.normalize();
	if(this->orientation.w() < 0.0) this->orientation = Eigen::Quaterniond(-this->orientation.w(), -this->orientation.x(), -this->orientation.y(), -this->orientation.z());
}

Pose::Pose(const Pose& p):
position(p.position), orientation(p.orientation)
{}
