#include "DynamicObstacleAvoidance/Modulation.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"
#include "DynamicObstacleAvoidance/Utils/Plotting/PlottingTools.hpp"
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

	for(int k=0; k<nb_simulations; ++k)
	{
		//unsigned int seed = rand() % 10000;
		unsigned int seed = 4298;
		srand(seed);

		Eigen::Vector3d position_o1 = Eigen::Vector3d::Random() * 2;
		Eigen::Vector3d position_o2 = Eigen::Vector3d::Random() * 2;
		Eigen::Vector3d position_o3 = Eigen::Vector3d::Random() * 2;
		position_o1(2) = 0;
		position_o2(2) = 0;
		position_o3(2) = 0;
		Eigen::Quaterniond orientation(1,0,0,0);

		auto ptrE1 = std::make_unique<Ellipsoid>(State(position_o1, orientation), 0.5);
		auto ptrE2 = std::make_unique<Ellipsoid>(State(position_o2, orientation), 0.5);
		auto ptrE3 = std::make_unique<Ellipsoid>(State(position_o3, orientation), 0.5);
		ptrE1->set_axis_lengths(Eigen::Array3d(1.1, 1.1, 0));
		ptrE2->set_axis_lengths(Eigen::Array3d(1, 1, 0));
		ptrE3->set_axis_lengths(Eigen::Array3d(1, 1, 0));

		std::deque<std::unique_ptr<Obstacle> > obstacle_list;
		obstacle_list.push_back(std::move(ptrE1));
		obstacle_list.push_back(std::move(ptrE2));
		obstacle_list.push_back(std::move(ptrE3));

		auto ptrAggregate = std::make_unique<Aggregate>(obstacle_list);
		std::deque<std::unique_ptr<Obstacle> > final_obstacle_list;
		final_obstacle_list.push_back(std::move(ptrAggregate));

		Eigen::Vector3d agent_position = Eigen::Vector3d::Random() * 5;
		agent_position(1) = 4;
		agent_position(2) = 0;
		State agent_state(agent_position);
		Agent agent(agent_state);

		Eigen::Vector3d target_position = Eigen::Vector3d::Random() * 5;
		target_position(1) = -4;
		target_position(2) = 0;
		std::deque<Eigen::Vector3d> position_history;

		for(int i=0; i<1000; ++i)
		{
			Eigen::Vector3d current_position = agent.get_position();
			position_history.push_back(current_position);

			Eigen::Vector3d desired_velocity = -Kp * (current_position - target_position);
			agent.set_linear_velocity(desired_velocity);

			Eigen::Vector3d modulated_velocity = Modulation::modulate_velocity(agent, final_obstacle_list);
			Eigen::Vector3d modulated_position = current_position + dt * modulated_velocity;
			agent.set_position(modulated_position);
		}

		PlottingTools::plot_configuration(agent, final_obstacle_list, target_position, position_history, "test_" + std::to_string(k) + "_seed_" + std::to_string(seed));
	}
}	