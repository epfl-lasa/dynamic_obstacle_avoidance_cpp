#include "DynamicObstacleAvoidance/Agent.hpp"

Agent::Agent(const State& state, const double& safety_margin):
state(state), safety_margin(safety_margin)
{}