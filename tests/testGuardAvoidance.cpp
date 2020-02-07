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
	//srand(seed);

	// generate the list of obstacles
	Eigen::Vector3d position_o1(0, 0, 0);
	Eigen::Quaterniond orientation_o6(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
	auto ptrE1 = std::make_shared<Ellipsoid>("w1", State(position_o1), 0.1);
	ptrE1->set_axis_lengths(Eigen::Array3d(1, 1, 1.5));
	ptrE1->set_curvature_factor(Eigen::Array3d(2, 2, 2));

	Environment env;
	env.add_obstacle(ptrE1);

	// create the target for agent
	Eigen::Vector3d target_position(5, 0.3, 0.5);
	std::deque<Eigen::Vector3d> position_history;

	// create the agent
	Eigen::Vector3d agent_position(-5, MathTools::rand_float(4,-4), 0.5);
	State agent_state(agent_position);
	Agent agent(agent_state);

	// control loop
	for(unsigned int i=0; i<nb_steps; ++i)
	{
		// agent
		Eigen::Vector3d current_position = agent.get_position();
		position_history.push_back(current_position);
		// compute agent velocity
		Eigen::Vector3d desired_velocity = -Kp * (current_position - target_position);
		//desired_velocity(1) = 0.0;
		agent.set_linear_velocity(desired_velocity);
		// compute modulated velocity
		Eigen::Vector3d modulated_velocity = Modulation::modulate_velocity(agent, env, false, true);
		if(modulated_velocity.norm() > max_vel) modulated_velocity = max_vel * modulated_velocity.normalized();
		// move agent
		Eigen::Vector3d modulated_position = current_position + dt * modulated_velocity;
		agent.set_position(modulated_position);

		if(plot_steps)
		{	
			std::stringstream ss;
			ss << std::setw(3) << std::setfill('0') << i;
			std::string s = ss.str();
			PlottingTools::plot_configuration(agent, env.get_obstacle_list(), target_position, position_history, "image" + s, is_show);
		}

		std::cout << agent << std::endl;
	}
	PlottingTools::plot_configuration(agent, env.get_obstacle_list(), target_position, position_history, "test_seed" + std::to_string(seed), is_show);
	//PlottingTools::plot_configuration(aggregated_obstacle_list, "test_seed" + std::to_string(seed), is_show);
}