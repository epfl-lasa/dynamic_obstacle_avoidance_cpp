#include "DynamicObstacleAvoidance/Agent.hpp"

Agent::Agent(const Agent& agent):
state(agent.state), safety_margin(agent.safety_margin)
{}

Agent::Agent(const State& state, const double& safety_margin):
state(state), safety_margin(safety_margin)
{}