#include "DynamicObstacleAvoidance/agent.hpp"

Agent::Agent(Eigen::VectorXf& position, Eigen::VectorXf& orientation, double safety_margin):
position(position), orientation(orientation), safety_margin(safety_margin)
{}