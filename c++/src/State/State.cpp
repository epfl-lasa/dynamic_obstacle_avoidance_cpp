#include "DynamicObstacleAvoidance/State/State.hpp"

State::State(const Pose& pose):pose(pose)
{}

State::State(const Eigen::Vector3f& position):pose(position)
{}

State::State(const Eigen::Vector3f& position, const Eigen::Quaternionf& orientation):
pose(position, orientation)
{}