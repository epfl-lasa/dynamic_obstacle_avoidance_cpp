#include "DynamicObstacleAvoidance/Modulation.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"
#include "DynamicObstacleAvoidance/Utils/Plotting/PlottingTools.hpp"
#include "DynamicObstacleAvoidance/Utils/ObstacleGeneration/Aggregation.hpp"
#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"
#include "DynamicObstacleAvoidance/Agent.hpp"
#include <eigen3/Eigen/Core>
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <string>
#include <sstream>
#include <iomanip>

using namespace DynamicObstacleAvoidance;

int main(int, char*[])
{
	srand(time(0));

	double Kp = 1;
	double dt = 0.01;

	unsigned int nb_steps = 400;
	bool is_show = false;
	bool plot_steps = true;
	double max_vel = 100;

	unsigned int seed = 679;
	srand(seed);

	// generate the list of obstacles
	Eigen::Vector3d position_o1(5, 0, 0);
	Eigen::Quaterniond orientation_o6(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
	auto ptrE1 = std::make_unique<Ellipsoid>(State(position_o1), 0.1, "w1");
	ptrE1->set_axis_lengths(Eigen::Array3d(1, 1, 0));

	std::deque<std::unique_ptr<Obstacle> > obstacle_list;
	obstacle_list.push_back(std::move(ptrE1));

	// create the target for agent
	Eigen::Vector3d target_position(0, 0.1, 0);
	std::deque<Eigen::Vector3d> position_history;
	// create target for obstacle
	Eigen::Vector3d obstacle_target_position(-5, 0, 0);

	std::deque<std::unique_ptr<Obstacle> > aggregated_obstacle_list;
	// aggregate the obstacles if necessary
	aggregated_obstacle_list = Aggregation::aggregate_obstacles(obstacle_list);

	// create the agent
	Eigen::Vector3d agent_position(0, 0, 0);
	State agent_state(agent_position);
	Agent agent(agent_state, 0.1);

	// control loop
	for(unsigned int i=0; i<nb_steps; ++i)
	{
		// agent
		Eigen::Vector3d current_position = agent.get_position();
		// obstacle
		Eigen::Vector3d obstacle_current_position = aggregated_obstacle_list[0]->get_position();

		position_history.push_back(current_position);
		// move agent
		Eigen::Vector3d desired_velocity = -Kp * (current_position - target_position);
		agent.set_linear_velocity(desired_velocity);
		// move obstacle
		Eigen::Vector3d obstacle_desired_velocity = -Kp * (obstacle_current_position - obstacle_target_position);
		aggregated_obstacle_list[0]->set_linear_velocity(obstacle_desired_velocity);

		Eigen::Vector3d modulated_velocity = Modulation::modulate_velocity(agent, aggregated_obstacle_list, false, true);
		if(modulated_velocity.norm() > max_vel) modulated_velocity = max_vel * modulated_velocity.normalized();

		// agent
		Eigen::Vector3d modulated_position = current_position + dt * modulated_velocity;
		agent.set_position(modulated_position);
		// obstacle
		Eigen::Vector3d obstacle_position = obstacle_current_position + dt * obstacle_desired_velocity;
		aggregated_obstacle_list[0]->set_position(obstacle_position);
		aggregated_obstacle_list[0]->set_reference_position(obstacle_position);

		if(plot_steps)
		{	
			std::stringstream ss;
			ss << std::setw(3) << std::setfill('0') << i;
			std::string s = ss.str();
			PlottingTools::plot_configuration(agent, aggregated_obstacle_list, target_position, position_history, "image" + s, is_show);
		}
	}
	PlottingTools::plot_configuration(agent, aggregated_obstacle_list, target_position, position_history, "test_seed" + std::to_string(seed), is_show);
	//PlottingTools::plot_configuration(aggregated_obstacle_list, "test_seed" + std::to_string(seed), is_show);
}