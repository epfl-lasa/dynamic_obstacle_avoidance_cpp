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
	bool plot_steps = false;
	double max_vel = 100;

	unsigned int seed = 679;
	srand(seed);

	// generate the list of obstacles
	Eigen::Vector3d position_o1(0, 8, 0);
	Eigen::Vector3d position_o2(0, -8, 0);
	Eigen::Vector3d position_o3(8, 0, 0);
	Eigen::Vector3d position_o4(-8, 0, 0);
	Eigen::Vector3d position_o5(1, -4, 0);
	Eigen::Vector3d position_o6(-3.5, 2.5, 0);
	Eigen::Vector3d position_o7(-3, 1.5, 0);
	Eigen::Vector3d position_o8(5, 0, 0);

	Eigen::Quaterniond orientation_o6(Eigen::AngleAxisd(-0.6, Eigen::Vector3d::UnitZ()));

	auto ptrE1 = std::make_shared<Ellipsoid>(State(position_o1), 0.1, "w1");
	auto ptrE2 = std::make_shared<Ellipsoid>(State(position_o2), 0.1, "w2");
	auto ptrE3 = std::make_shared<Ellipsoid>(State(position_o3), 0.1, "w3");
	auto ptrE4 = std::make_shared<Ellipsoid>(State(position_o4), 0.1, "w4");
	auto ptrE5 = std::make_shared<Ellipsoid>(State(position_o5), 0.5, "w5");
	auto ptrE6 = std::make_shared<Ellipsoid>(State(position_o6, orientation_o6),  0.1, "desk");
	auto ptrE7 = std::make_shared<Ellipsoid>(State(position_o7), 0.1, "chair");
	auto ptrE8 = std::make_shared<Ellipsoid>(State(position_o8), 0.1, "table");

	ptrE1->set_axis_lengths(Eigen::Array3d(9, 1, 0));
	ptrE2->set_axis_lengths(Eigen::Array3d(9, 1, 0));
	ptrE3->set_axis_lengths(Eigen::Array3d(1, 9, 0));
	ptrE4->set_axis_lengths(Eigen::Array3d(1, 9, 0));
	ptrE5->set_axis_lengths(Eigen::Array3d(1, 4.5, 0));

	ptrE6->set_axis_lengths(Eigen::Array3d(0.5, 3, 0));
	ptrE7->set_axis_lengths(Eigen::Array3d(1, 1, 0));

	ptrE8->set_axis_lengths(Eigen::Array3d(2, 2, 0));

	std::deque<std::shared_ptr<Obstacle> > obstacle_list;
	obstacle_list.push_back(ptrE1);
	obstacle_list.push_back(ptrE2);
	obstacle_list.push_back(ptrE3);
	obstacle_list.push_back(ptrE4);
	obstacle_list.push_back(ptrE5);
	obstacle_list.push_back(ptrE6);
	obstacle_list.push_back(ptrE7);
	obstacle_list.push_back(ptrE8);

	// create the target
	Eigen::Vector3d target_position(5, -5 , 0);
	std::deque<Eigen::Vector3d> position_history;

	std::deque<std::shared_ptr<Obstacle> > aggregated_obstacle_list;

	// aggregate the obstacles if necessary
	aggregated_obstacle_list = Aggregation::aggregate_obstacles(obstacle_list);
	aggregated_obstacle_list[0]->set_reference_position(Eigen::Vector3d(0, 0, 0));
	static_cast<Aggregate*>(aggregated_obstacle_list[0].get())->update_hull();

	/*ptrE5->set_reference_position(Eigen::Vector3d(1,-7.5,0));
	aggregated_obstacle_list.push_back(ptrE5));

	ptrE8->set_reference_position(Eigen::Vector3d(7.5,0,0));
	aggregated_obstacle_list.push_back(ptrE8));*/
	
	//Eigen::Vector3d object_target_position(-2, 8, 0);
	Eigen::Vector3d object_target_position(1.5, 0, 0);

	// create the agent
	Eigen::Vector3d agent_position(-5, -5, 0);

	State agent_state(agent_position);
	Agent agent(agent_state, 0.1);

	// control loop
	for(unsigned int i=0; i<nb_steps; ++i)
	{
		Eigen::Vector3d current_position = agent.get_position();

		// move object
		//Eigen::Vector3d object_current_position = obstacle_list[2]->get_position();
		//Eigen::Vector3d object_desired_velocity = -Kp * (object_current_position - object_target_position);
		//obstacle_list[2]->set_position(object_current_position + dt * object_desired_velocity);
		//obstacle_list[2]->set_reference_position(obstacle_list[2]->get_position());

		// aggregate the obstacles if necessary
		//aggregated_obstacle_list = Aggregation::aggregate_obstacles(obstacle_list);
		//aggregated_obstacle_list[0]->set_reference_position(aggregate_reference_position);
		//static_cast<Aggregate*>(aggregated_obstacle_list[0].get())->update_hull();

		Eigen::Vector3d previous_vel = Eigen::Vector3d::Zero();
		if(agent.exist_path(target_position, aggregated_obstacle_list))
		{
			position_history.push_back(current_position);

			Eigen::Vector3d desired_velocity = -Kp * (current_position - target_position);
			//agent.set_linear_velocity(0.9 * desired_velocity + 0.1 * previous_vel);
			agent.set_linear_velocity(desired_velocity);

			Eigen::Vector3d modulated_velocity = Modulation::modulate_velocity(agent, aggregated_obstacle_list, false, true);
			if(modulated_velocity.norm() > max_vel) modulated_velocity = max_vel * modulated_velocity.normalized();

			Eigen::Vector3d modulated_position = current_position + dt * modulated_velocity;
			agent.set_position(modulated_position);

			previous_vel = desired_velocity;
		}
		else
		{
			//std::cout << "No path to target!" << std::endl;
			previous_vel = Eigen::Vector3d::Zero();
			agent.set_linear_velocity(previous_vel);
		}

		if(plot_steps)
		{	
			std::stringstream ss;
			ss << std::setw(3) << std::setfill('0') << i;
			std::string s = ss.str();
			PlottingTools::plot_configuration(agent, aggregated_obstacle_list, target_position, position_history, "image" + s, is_show);
		}
	}
	//PlottingTools::plot_configuration(agent, aggregated_obstacle_list, target_position, position_history, "test_seed" + std::to_string(seed), is_show);
	PlottingTools::plot_configuration(aggregated_obstacle_list, "test_seed" + std::to_string(seed), is_show);
}