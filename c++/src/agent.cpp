#include "DynamicObstacleAvoidance/agent.hpp"

Agent::Agent(Eigen::Vector3f& position, Eigen::Vector4f& orientation, double safety_margin):
position(position), orientation(orientation), safety_margin(safety_margin)
{}