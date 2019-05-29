#include "DynamicObstacleAvoidance/State/State.hpp"

State::State():
linear_velocity(0,0,0), angular_velocity(0,0,0)
{}

State::State(const Pose& pose):
pose(pose), linear_velocity(0,0,0), angular_velocity(0,0,0)
{}

State::State(const double& x, const double& y, const double& z):
pose(x,y,z), linear_velocity(0,0,0), angular_velocity(0,0,0)
{}

State::State(const Eigen::Vector3d& position):
pose(position), linear_velocity(0,0,0), angular_velocity(0,0,0)
{}

State::State(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation):
pose(position, orientation), linear_velocity(0,0,0), angular_velocity(0,0,0)
{}

State::State(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity):
pose(position, orientation), linear_velocity(linear_velocity), angular_velocity(angular_velocity)
{}