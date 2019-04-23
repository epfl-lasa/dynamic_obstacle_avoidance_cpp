#include "DynamicObstacleAvoidance/modulation.hpp"

namespace Modulation
{
	Eigen::Vector3f compute_normal_to_obstacle(const Obstacle& obstacle, const Eigen::Vector3f& agent_position)
	{
		// the normal to the obstacle is the opposite of the normal to the agent in the obstacle reference frame
		return (-1) * obstacle.compute_normal_to_external_point(agent_position);
	}

	Eigen::Vector3f compute_direction_to_obstacle(const Obstacle& obstacle, const Eigen::Vector3f& agent_position)
	{
		return obstacle.get_reference_position() - agent_position;
	}

	double compute_distance_to_obstacle(const Obstacle& obstacle, const Eigen::Vector3f& agent_position)
	{
		return obstacle.compute_distance_to_external_point(agent_position);
	}

	double compute_modulation_matrix(const Agent& agent, const Obstacle& obstacle, Eigen::MatrixXf& basis, Eigen::MatrixXf& orthogonal_basis, Eigen::MatrixXf& eigenvalues)
	{
		Eigen::Vector3f agent_position = agent.get_position();
		// compute all necessary vectors
		Eigen::Vector3f normal_to_obstacle = compute_normal_to_obstacle(obstacle, agent_position);
		Eigen::Vector3f direction_to_obstacle = compute_direction_to_obstacle(obstacle, agent_position);
		double distance_to_obstacle = compute_distance_to_obstacle(obstacle, agent_position);

		return distance_to_obstacle;
	}

	Eigen::ArrayXf weight_obstacles(Eigen::ArrayXf& distances, double critical_distance, double weight_power)
	{
		Eigen::ArrayXf weights(distances.size());
		Eigen::ArrayXf critical_obstacles(distances.size());

		int sum_critical_obstacles = 0;
		for(int i=0; i<distances.size(); ++i) 
		{
			int tmp_value = (distances(i) <= critical_distance) ? 1 : 0;
			critical_obstacles(i) = tmp_value;
			sum_critical_obstacles += tmp_value;
		}

		if(sum_critical_obstacles > 0)
		{
			weights = critical_obstacles / sum_critical_obstacles;
		}
		else
		{
			weights = Eigen::pow(distances - critical_distance, -weight_power);
			weights /= weights.sum();
		}
		return weights;
	}

	Eigen::Vector3f modulate_velocity(const Agent& agent, const std::list<Obstacle>& obstacles, const std::list<Eigen::Vector3f>& attractor_positions)
	{
		if(obstacles.empty()) return agent.get_velocity();
		int dim = agent.get_position().size();

		// initialize the list of matrices for calculation
		std::list<Eigen::MatrixXf> basis_list;
		std::list<Eigen::MatrixXf> orthogonal_basis_list;
		std::list<Eigen::MatrixXf> eigenvalues_list;
		Eigen::ArrayXf distances(obstacles.size());

		// compute all necessary elements to calculation the modulation matrix
		int k = 0;
		for(const Obstacle& obs:obstacles)
		{
			Eigen::MatrixXf basis(dim, dim);
			Eigen::MatrixXf orthogonal_basis(dim, dim);
			Eigen::MatrixXf eigenvalues(dim, dim);

			double distance = compute_modulation_matrix(agent, obs, basis, orthogonal_basis, eigenvalues);
			basis_list.push_back(basis);
			orthogonal_basis_list.push_back(orthogonal_basis);
			eigenvalues_list.push_back(eigenvalues);
			
			distances(k) = distance;
			++k;
		}
		Eigen::ArrayXf weights = Modulation::weight_obstacles(distances, 1.0, 2.0);

	}
}