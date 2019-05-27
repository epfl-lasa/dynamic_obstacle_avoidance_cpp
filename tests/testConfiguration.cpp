#include "DynamicObstacleAvoidance/Modulation.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"
#include "DynamicObstacleAvoidance/Utils/Plotting/PlottingTools.hpp"
#include "DynamicObstacleAvoidance/Utils/ObstacleGeneration/Aggregation.hpp"
#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"
#include <eigen3/Eigen/Core>
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <string>

int main(int, char*[])
{
	srand(time(0));

	double Kp = 1;
	double dt = 0.01;

	int nb_simulations = 100;
	int nb_steps = 1000;
	bool is_show = false;
	bool debug = false;

	for(int k=0; k<nb_simulations; ++k)
	{
		std::cerr << k << std::endl; 
		unsigned int seed = rand() % 10000;
		//unsigned int seed = 8698;
		srand(seed);

		// generate the list of obstacles
		Eigen::Vector3d position_o1(MathTools::rand_float(3,-3), MathTools::rand_float(3,-3), 0);
		Eigen::Vector3d position_o2(MathTools::rand_float(3,-3), MathTools::rand_float(3,-3), 0);
		Eigen::Vector3d position_o3(MathTools::rand_float(3,-3), MathTools::rand_float(3,-3), 0);

		Eigen::Quaterniond orientation_o1(Eigen::AngleAxisd(MathTools::rand_float(2)*M_PI, Eigen::Vector3d::UnitZ()));
		Eigen::Quaterniond orientation_o2(Eigen::AngleAxisd(MathTools::rand_float(2)*M_PI, Eigen::Vector3d::UnitZ()));
		Eigen::Quaterniond orientation_o3(Eigen::AngleAxisd(MathTools::rand_float(2)*M_PI, Eigen::Vector3d::UnitZ()));

/*		orientation_o1 = Eigen::Quaterniond(1,0,0,0);
		orientation_o2 = Eigen::Quaterniond(1,0,0,0);
		orientation_o3 = Eigen::Quaterniond(1,0,0,0);*/

		auto ptrE1 = std::make_unique<Ellipsoid>(State(position_o1, orientation_o1), MathTools::rand_float(1));
		auto ptrE2 = std::make_unique<Ellipsoid>(State(position_o2, orientation_o2), MathTools::rand_float(1));
		auto ptrE3 = std::make_unique<Ellipsoid>(State(position_o3, orientation_o3), MathTools::rand_float(1));


		ptrE1->set_axis_lengths(Eigen::Array3d(MathTools::rand_float(2), MathTools::rand_float(2), 0));
		ptrE2->set_axis_lengths(Eigen::Array3d(MathTools::rand_float(2), MathTools::rand_float(2), 0));
		ptrE3->set_axis_lengths(Eigen::Array3d(MathTools::rand_float(2), MathTools::rand_float(2), 0));

		std::deque<std::unique_ptr<Obstacle> > obstacle_list;
		obstacle_list.push_back(std::move(ptrE1));
		obstacle_list.push_back(std::move(ptrE2));
		obstacle_list.push_back(std::move(ptrE3));

		// aggregate the obstacles if necessary
		std::deque<std::unique_ptr<Obstacle> > aggregated_obstacle_list = Aggregation::aggregate_obstacles(obstacle_list);

		// create the agent
		Eigen::Vector3d agent_position(MathTools::rand_float(5, -5), 8, 0);
		State agent_state(agent_position);
		Agent agent(agent_state);

		// create the target
		Eigen::Vector3d target_position(MathTools::rand_float(5, -5), -8, 0);
		std::deque<Eigen::Vector3d> position_history;

		if(debug) 
		{
			for(auto& o:aggregated_obstacle_list) std::cerr << *o << std::endl;
			std::cerr << std::endl;
			std::cerr << agent << std::endl;
			std::cerr << "target: (" << target_position(0) << ", ";
			std::cerr << target_position(1) << ", ";
			std::cerr << target_position(2) << ")" << std::endl;
		}

		// control loop
		for(int i=0; i<nb_steps; ++i)
		{
			Eigen::Vector3d current_position = agent.get_position();
			position_history.push_back(current_position);

			Eigen::Vector3d desired_velocity = -Kp * (current_position - target_position);
			agent.set_linear_velocity(desired_velocity);

			Eigen::Vector3d modulated_velocity = Modulation::modulate_velocity(agent, aggregated_obstacle_list);
			Eigen::Vector3d modulated_position = current_position + dt * modulated_velocity;
			agent.set_position(modulated_position);
		}

		PlottingTools::plot_configuration(agent, aggregated_obstacle_list, target_position, position_history, "test_" + std::to_string(k) + "_seed_" + std::to_string(seed), is_show);
	}
}	