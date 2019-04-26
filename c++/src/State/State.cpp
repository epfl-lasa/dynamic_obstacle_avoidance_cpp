#include "DynamicObstacleAvoidance/State/State.hpp"

State::State(const Pose& pose):
pose(pose), linear_velocity(0,0,0), angular_velocity(0,0,0)
{}

State::State(const Eigen::Vector3f& position):
pose(position), linear_velocity(0,0,0), angular_velocity(0,0,0)
{}

State::State(const Eigen::Vector3f& position, const Eigen::Quaternionf& orientation):
pose(position, orientation), linear_velocity(0,0,0), angular_velocity(0,0,0)
{}