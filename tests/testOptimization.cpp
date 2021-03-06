#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Aggregate.hpp"
#include "DynamicObstacleAvoidance/Obstacle/StarShapeHull.hpp"
#include "DynamicObstacleAvoidance/Utils/Plotting/PlottingTools.hpp"
#include "DynamicObstacleAvoidance/Utils/ObstacleGeneration/Aggregation.hpp"
#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"
#include "DynamicObstacleAvoidance/Agent.hpp"
#include <eigen3/Eigen/Core>
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <string>

using namespace DynamicObstacleAvoidance;

int main(int, char*[])
{
	bool is_show = true;
	double obstacles_safety_margin = 0.0;
	double agent_safety_margin = 0.0;

	Eigen::Vector3d agent_position(2, 5, 0.0);
	Eigen::Vector3d target_position(4.18, 3.84, 0.);

	// create the agent
	State agent_state(agent_position);
	Agent agent(agent_state, agent_safety_margin);

	// generate the list of obstacles
	Eigen::Vector3d position_o1(2.95, 2.5, 0);
	Eigen::Vector3d position_o2(4.4, 2, 0);
	Eigen::Vector3d position_o3(0.3, 3, 0);
	Eigen::Vector3d position_o4(1, 3.75, 0);
	Eigen::Vector3d position_o5(3.7, 4.7, 0);
	Eigen::Vector3d position_o6(4.6, 4.7, 0);
	Eigen::Vector3d position_o7(2, 5.7, 0);

	Eigen::Vector3d position_o8(0.5, 3.5, 0);


	Eigen::Quaterniond orientation_o1(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o2(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o3(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o4(Eigen::AngleAxisd(0.38, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o5(Eigen::AngleAxisd(0.75, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o6(Eigen::AngleAxisd(-0.75, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o7(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o8(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));


	std::shared_ptr<Ellipsoid> ptrE1 = std::make_shared<Ellipsoid>("e1", State(position_o1, orientation_o1), obstacles_safety_margin);
	std::shared_ptr<Ellipsoid> ptrE2 = std::make_shared<Ellipsoid>("e2", State(position_o2, orientation_o2), obstacles_safety_margin);
	std::shared_ptr<Ellipsoid> ptrE3 = std::make_shared<Ellipsoid>("e3", State(position_o3, orientation_o3), obstacles_safety_margin);
	std::shared_ptr<Ellipsoid> ptrE4 = std::make_shared<Ellipsoid>("e4", State(position_o4, orientation_o4), obstacles_safety_margin);
	std::shared_ptr<Ellipsoid> ptrE5 = std::make_shared<Ellipsoid>("e5", State(position_o5, orientation_o5), obstacles_safety_margin);
	std::shared_ptr<Ellipsoid> ptrE6 = std::make_shared<Ellipsoid>("e6", State(position_o6, orientation_o6), obstacles_safety_margin);
	std::shared_ptr<Ellipsoid> ptrE7 = std::make_shared<Ellipsoid>("e7", State(position_o7, orientation_o7), obstacles_safety_margin);
	std::shared_ptr<Ellipsoid> ptrE8 = std::make_shared<Ellipsoid>("e8", State(position_o8, orientation_o8), obstacles_safety_margin);

	ptrE1->set_axis_lengths(Eigen::Array3d(0.2, 0.75, 0));
	ptrE2->set_axis_lengths(Eigen::Array3d(1.5, 0.15, 0));
	ptrE3->set_axis_lengths(Eigen::Array3d(0.2, 2, 0));
	ptrE4->set_axis_lengths(Eigen::Array3d(0.7, 0.3, 0));
	ptrE5->set_axis_lengths(Eigen::Array3d(0.65, 0.3, 0));
	ptrE6->set_axis_lengths(Eigen::Array3d(0.55, 0.2, 0));
	ptrE7->set_axis_lengths(Eigen::Array3d(2, 0.2, 0));
	ptrE8->set_axis_lengths(Eigen::Array3d(0.5, 0.5, 0));

	// add to the list
	std::deque<std::shared_ptr<Obstacle> > obstacle_list;
	obstacle_list.push_back(ptrE1);
	obstacle_list.push_back(ptrE2);
	obstacle_list.push_back(ptrE3);
	obstacle_list.push_back(ptrE4);
	obstacle_list.push_back(ptrE5);
	obstacle_list.push_back(ptrE6);
	obstacle_list.push_back(ptrE7);
	obstacle_list.push_back(ptrE8);

	// aggregate the obstacles if necessary
	std::deque<std::shared_ptr<Obstacle> > aggregated_obstacle_list = Aggregation::aggregate_obstacles(obstacle_list);

	// plot the configuration
	std::deque<Eigen::Vector3d> position_history;
	PlottingTools::plot_configuration(agent, aggregated_obstacle_list, target_position, position_history, "test", is_show);
}