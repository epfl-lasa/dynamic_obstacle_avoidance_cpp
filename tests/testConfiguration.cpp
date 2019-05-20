#include "DynamicObstacleAvoidance/Modulation.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Utils/Plotting/PlottingTools.hpp"
#include <eigen3/Eigen/Core>


int main(int, char*[])
{
	double Kp = 1;
	double dt = 0.01;

	Eigen::Vector3d position_o1(-1, 1, 0);
	Eigen::Vector3d position_o2(0.5, -0.5, 0);
	Eigen::Quaterniond orientation(1,0,0,0);

	auto ptrE1 = std::make_unique<Ellipsoid>(State(position_o1, orientation), 0.5);
	auto ptrE2 = std::make_unique<Ellipsoid>(State(position_o2, orientation), 0.5);
	ptrE1->set_axis_lengths(Eigen::Array3d(1.2, 1.2, 1));
	ptrE2->set_axis_lengths(Eigen::Array3d(1.2, 1.2, 1));

	std::deque<std::unique_ptr<Obstacle> > obstacle_list;
	obstacle_list.push_back(std::move(ptrE1));
	obstacle_list.push_back(std::move(ptrE2));

	Eigen::Vector3d agent_position(0,4,0);
	State agent_state(agent_position);
	Agent agent(agent_state);

	Eigen::Vector3d target_position(-1,-4,0);
	std::deque<Eigen::Vector3d> position_history;

	for(int i=0; i<1000; ++i)
	{
		Eigen::Vector3d current_position = agent.get_position();
		position_history.push_back(current_position);

		Eigen::Vector3d desired_velocity = -Kp * (current_position - target_position);
		agent.set_linear_velocity(desired_velocity);

		Eigen::Vector3d modulated_velocity = Modulation::modulate_velocity(agent, obstacle_list);
		Eigen::Vector3d modulated_position = current_position + dt * modulated_velocity;
		agent.set_position(modulated_position);
	}

	PlottingTools::plot_configuration(agent, obstacle_list, target_position, position_history);
}	