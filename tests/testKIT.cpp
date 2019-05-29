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

int main(int, char*[])
{
	srand(time(0));

	double Kp = 1;
	double dt = 0.01;

	int nb_simulations = 1;
	int nb_steps = 1000;
	bool debug = false;
	bool is_show = false;
	bool plot_steps = true;

	// generate the list of obstacles
	Eigen::Vector3d position_o1(3, 2.62, 0);
	Eigen::Vector3d position_o2(4.4, 2, 0);
	Eigen::Vector3d position_o3(0.3, 3, 0);
	Eigen::Vector3d position_o4(1, 3.75, 0);
	Eigen::Vector3d position_o5(3.7, 4.7, 0);
	Eigen::Vector3d position_o6(4.6, 4.7, 0);
	Eigen::Vector3d position_o7(2, 5.7, 0);

	Eigen::Quaterniond orientation_o1(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o2(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o3(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o4(Eigen::AngleAxisd(0.38, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o5(Eigen::AngleAxisd(0.75, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o6(Eigen::AngleAxisd(-0.75, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o7(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));

	auto ptrE1 = std::make_unique<Ellipsoid>(State(position_o1, orientation_o1));
	auto ptrE2 = std::make_unique<Ellipsoid>(State(position_o2, orientation_o2));
	auto ptrE3 = std::make_unique<Ellipsoid>(State(position_o3, orientation_o3));
	auto ptrE4 = std::make_unique<Ellipsoid>(State(position_o4, orientation_o4));
	auto ptrE5 = std::make_unique<Ellipsoid>(State(position_o5, orientation_o5));
	auto ptrE6 = std::make_unique<Ellipsoid>(State(position_o6, orientation_o6));
	auto ptrE7 = std::make_unique<Ellipsoid>(State(position_o7, orientation_o7));

	ptrE1->set_axis_lengths(Eigen::Array3d(0.05, 0.7, 0));
	ptrE2->set_axis_lengths(Eigen::Array3d(1.5, 0.15, 0));
	ptrE3->set_axis_lengths(Eigen::Array3d(0.2, 2, 0));
	ptrE4->set_axis_lengths(Eigen::Array3d(0.7, 0.2, 0));
	ptrE5->set_axis_lengths(Eigen::Array3d(0.75, 0.2, 0));
	ptrE6->set_axis_lengths(Eigen::Array3d(0.75, 0.2, 0));
	ptrE7->set_axis_lengths(Eigen::Array3d(2, 0.2, 0));

	// set reference points for pairs of ellipses
	ptrE1->set_reference_position(Eigen::Vector3d(3, 2.05, 0));
	ptrE2->set_reference_position(Eigen::Vector3d(3, 2.05, 0));

	ptrE3->set_reference_position(Eigen::Vector3d(0.4, 3.55, 0));
	ptrE4->set_reference_position(Eigen::Vector3d(0.4, 3.55, 0));

	ptrE5->set_reference_position(Eigen::Vector3d(4.15, 5.1, 0));
	ptrE6->set_reference_position(Eigen::Vector3d(4.15, 5.1, 0));

	// add to the list
	std::deque<std::unique_ptr<Obstacle> > obstacle_list;
	obstacle_list.push_back(std::move(ptrE1));
	obstacle_list.push_back(std::move(ptrE2));
	obstacle_list.push_back(std::move(ptrE3));
	obstacle_list.push_back(std::move(ptrE4));
	obstacle_list.push_back(std::move(ptrE5));
	obstacle_list.push_back(std::move(ptrE6));
	//obstacle_list.push_back(std::move(ptrE7));

	for(int k=0; k<nb_simulations; ++k)
	{
		std::cerr << k << std::endl; 
		unsigned int seed = rand() % 10000;
		//unsigned int seed = 8698;
		srand(seed);

		// create the agent
		Eigen::Vector3d agent_position(MathTools::rand_float(6,4), MathTools::rand_float(3,1), 0);
		State agent_state(agent_position);
		Agent agent(agent_state, 0.5);

		// create the target
		//Eigen::Vector3d target_position(MathTools::rand_float(2.5,1), MathTools::rand_float(3), 0);
		Eigen::Vector3d target_position(MathTools::rand_float(2.5,1.5), MathTools::rand_float(6,4), 0);

		std::deque<Eigen::Vector3d> position_history;

		if(debug) 
		{
			for(auto& o:obstacle_list) std::cerr << *o << std::endl;
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

			Eigen::Vector3d modulated_velocity = Modulation::modulate_velocity(agent, obstacle_list);
			Eigen::Vector3d modulated_position = current_position + dt * modulated_velocity;
			agent.set_position(modulated_position);
			
			if(plot_steps)
			{	
				PlottingTools::plot_configuration(agent, obstacle_list, target_position, position_history, "test_" + std::to_string(k) + "_seed_" + std::to_string(seed) + "_step_" + std::to_string(i), is_show);
			}
		}

		PlottingTools::plot_configuration(agent, obstacle_list, target_position, position_history, "test_" + std::to_string(k) + "_seed_" + std::to_string(seed), is_show);
	}
}