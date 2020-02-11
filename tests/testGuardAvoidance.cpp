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
#include <random>

using namespace DynamicObstacleAvoidance;

int main(int, char*[])
{
	srand(time(0));
	std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> noise{0, 0.01};

	double Kp = 1;
	double dt = 0.01;

	unsigned int nb_steps = 400;
	bool is_show = false;
	bool plot_steps = true;
	double guard_max_vel = 100;
	double platform_max_vel = 100;

	bool local_modulation = false;
	bool add_repulsion = true;
	bool planar_modulation = true;

	// generate the list of obstacles for the platform
	Eigen::Vector3d position_o1(0, 0, 0);
	Eigen::Quaterniond orientation_o1(Eigen::AngleAxisd(0., Eigen::Vector3d::UnitZ()));
	auto ptrE1 = std::make_shared<Ellipsoid>("platform_obstacle", State(position_o1, orientation_o1), 0.5);
	ptrE1->set_axis_lengths(Eigen::Array3d(1.0, 1.0, 1.0));
	ptrE1->set_curvature_factor(Eigen::Array3d(2, 2, 2));

	// generate the list of obstacles for the guard (we use different safety margins to represent the guard)
	auto ptrE2 = std::make_shared<Ellipsoid>("guard_obstacle", State(position_o1, orientation_o1), Eigen::Array3d(0.5, 1.0, 0.2));
	ptrE2->set_axis_lengths(Eigen::Array3d(0.5, 0.5, 1.0));
	ptrE2->set_curvature_factor(Eigen::Array3d(2, 2, 2));

	Environment platform_env;
	platform_env.add_obstacle(ptrE1);
	Environment guard_env;
	guard_env.add_obstacle(ptrE2);

	// create the target for guard
	Eigen::Vector3d guard_target_position(5 + noise(gen), noise(gen), 0.4 + noise(gen));
	std::deque<Eigen::Vector3d> position_history;

	// create the moving platform
	Eigen::Vector3d platform_position(-5 + noise(gen), -1 + noise(gen), 0.0);
	State platform_state(platform_position);
	Agent platform(platform_state);
	// create the guard (with position in the platform reference frame)
	Eigen::Vector3d guard_fixed_transform(0, 1, 0.2);
	Eigen::Vector3d guard_position = platform.get_pose() * guard_fixed_transform; 
	State guard_state(guard_position);
	Agent guard(guard_state);

	// control loop
	for(unsigned int i=0; i<nb_steps; ++i)
	{
		// platform 
		Eigen::Vector3d current_platform_position = platform.get_position();
		position_history.push_back(current_platform_position);
		// guard
		Eigen::Vector3d current_guard_position = guard.get_position();
		
		// compute guard velocity
		Eigen::Vector3d desired_guard_velocity = -Kp * (current_guard_position - guard_target_position);
		//desired_velocity(1) = 0.0;
		guard.set_linear_velocity(desired_guard_velocity);
		// compute modulated velocity
		Eigen::Vector3d guard_modulated_velocity = Modulation::modulate_velocity(guard, guard_env, local_modulation, add_repulsion, planar_modulation);
		if(guard_modulated_velocity.norm() > guard_max_vel) guard_modulated_velocity = guard_max_vel * guard_modulated_velocity.normalized();
		
		// extract x, y velocities to feed to the platform modulation
		Eigen::Vector3d platform_velocity = guard_modulated_velocity;
		platform_velocity(2) = 0;
		platform.set_linear_velocity(platform_velocity);
		// perform modulation of the platform
		Eigen::Vector3d platform_modulated_velocity = Modulation::modulate_velocity(platform, platform_env, local_modulation, add_repulsion, false);
		if(platform_modulated_velocity.norm() > platform_max_vel) platform_modulated_velocity = platform_max_vel * platform_modulated_velocity.normalized();
		
		// move both agents starting by the platform
		Eigen::Vector3d platform_modulated_position = current_platform_position + dt * platform_modulated_velocity + Eigen::Vector3d(noise(gen), noise(gen), noise(gen));
		platform.set_position(platform_modulated_position);
		// move only the guard in z and get its x and y from the platform
		Eigen::Vector3d guard_modulated_position = platform.get_pose() * guard_fixed_transform;
		guard_modulated_position(2) = current_guard_position(2) + dt * guard_modulated_velocity(2) + noise(gen);
		guard.set_position(guard_modulated_position);
		
		if(plot_steps)
		{	
			std::stringstream ss;
			ss << std::setw(3) << std::setfill('0') << i;
			std::string s = ss.str();
			PlottingTools::plot_guard_configuration(platform, guard, platform_env.get_obstacle_list(), guard_target_position, position_history, "image" + s, is_show);
		}
	}
}