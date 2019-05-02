#include "DynamicObstacleAvoidance/State/Pose.hpp"

Pose::Pose():position(0,0,0), orientation(1,0,0,0)
{}

Pose::Pose(const Eigen::Vector3f& position):position(position), orientation(1,0,0,0)
{}

Pose::Pose(const Eigen::Vector3f& position, const Eigen::Quaternionf& orientation):
position(position), orientation(orientation)
{
	if(abs(orientation.norm()) - 1 < 1e-4) this->orientation.normalize();
}

Pose::Pose(const float& x, const float& y, const float& z):
position(x,y,z), orientation(1,0,0,0)
{}

Pose::Pose(const float& px, const float& py, const float& pz, const float& qw, const float& qx, const float& qy, const float& qz):
position(px,py,pz), orientation(qw,qx,qy,qz)
{
	if(abs(orientation.norm()) - 1 < 1e-4) this->orientation.normalize();
}

Pose::Pose(const Pose& p):
position(p.position), orientation(p.orientation)
{}
