#include "DynamicObstacleAvoidance/Modulation.hpp"

namespace Modulation
{
	std::tuple<Eigen::Matrix3f, Eigen::Matrix3f, float> compute_modulation_matrix(const Agent& agent, const Obstacle& obstacle)
	{
		Eigen::Vector3f agent_position = agent.get_position();

		// compute all necessary vectors
		Eigen::Vector3f normal_vector = obstacle.compute_normal_to_external_point(agent_position);
		float distance_to_obstacle = obstacle.compute_distance_to_external_point(agent_position);

		// compute the basis matrices
		auto basis_matrices = compute_basis_matrices(normal_vector, agent_position, obstacle.get_reference_position());
		Eigen::Matrix3f reference_basis = std::get<0>(basis_matrices);
		Eigen::Matrix3f orthogonal_basis = std::get<1>(basis_matrices);

		// compute the diagonal eigenvalues
		Eigen::DiagonalMatrix<float, 3> diagonal_eigenvalues = compute_diagonal_eigenvalues(obstacle, distance_to_obstacle);

		// compute the modulation matrix
		Eigen::Matrix3f modulation_matrix = obstacle.get_orientation() * reference_basis * diagonal_eigenvalues * reference_basis.inverse();
		// return all computed elements
		return std::make_tuple(modulation_matrix, orthogonal_basis, distance_to_obstacle);
	}

	Eigen::ArrayXf weight_obstacles(const Eigen::ArrayXf& distances, const float& critical_distance, const float& weight_power)
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

	Eigen::DiagonalMatrix<float, 3> compute_diagonal_eigenvalues(const Obstacle& obstacle, const float& distance_to_obstacle, const float& reactivity_factor)
	{
		// check if the point is inside the obstacle
		float delta_eigenvalue = (distance_to_obstacle <= 1) ? 1.0 : 1.0 / std::pow(distance_to_obstacle, 1.0 / reactivity_factor);
		float eigenvalue_reference = 1 - delta_eigenvalue;
		float eigenvalue_tangent = 1 + delta_eigenvalue;
		Eigen::DiagonalMatrix<float, 3> diagonal_eigenvalues(eigenvalue_reference, eigenvalue_tangent, eigenvalue_tangent);
		return diagonal_eigenvalues;
	}

	std::pair<Eigen::Matrix3f, Eigen::Matrix3f> compute_basis_matrices(const Eigen::Vector3f& normal_vector, const Eigen::Vector3f& agent_position, const Eigen::Vector3f& obstacle_reference_position)
	{
		Eigen::Vector3f unit_vector = Eigen::Vector3f::UnitZ();
		Eigen::Vector3f tangent_vector = normal_vector.cross(unit_vector);
		
		if(tangent_vector.norm() == 0) 
		{
			unit_vector = Eigen::Vector3f::UnitY();
			tangent_vector = normal_vector.cross(unit_vector);
		}

		Eigen::Vector3f cross_product = tangent_vector.cross(normal_vector);
		Eigen::Matrix3f orthogonal_basis;
		orthogonal_basis << normal_vector, tangent_vector, cross_product;

		Eigen::Vector3f reference_direction = agent_position - obstacle_reference_position;
		reference_direction.normalize();
		Eigen::Matrix3f reference_basis;
		reference_basis << reference_direction, tangent_vector, cross_product;
		return std::make_pair(reference_basis, orthogonal_basis);
	}

	Eigen::Vector3f compute_relative_velocity(const Agent& agent, const std::deque<std::unique_ptr<Obstacle> >& obstacles, const Eigen::ArrayXf& distances, const Eigen::ArrayXf& weights, const std::deque<Eigen::Matrix3f>& orthogonal_basis_list)
	{
		Eigen::Vector3f obs_velocity(0,0,0);
		int i = 0;
		for(auto &obs:obstacles)
		{
			Eigen::Vector3f angular_velocity = obs->get_angular_velocity().cross(agent.get_position() - obs->get_position());
			float exp_weight = exp(-(std::max<float>(1.0, distances(i))-1));
			Eigen::Vector3f obs_velocity_tmp = exp_weight * (obs->get_linear_velocity() + angular_velocity);
			obs_velocity_tmp = orthogonal_basis_list[i].transpose() * obs_velocity_tmp;
			// remove negative values in the orthogonal part
			obs_velocity_tmp(0) = std::max<float>(0.0, obs_velocity_tmp(0));
			obs_velocity_tmp = orthogonal_basis_list[i] * obs_velocity_tmp;
			// sum to get the velocity of all obstackes
			obs_velocity += obs_velocity_tmp * weights(i);
			++i;
		}
		return agent.get_linear_velocity() - obs_velocity;
	}



	Eigen::Vector3f modulate_velocity(const Agent& agent, const std::deque<std::unique_ptr<Obstacle> >& obstacles, const std::deque<Eigen::Vector3f>& attractor_positions)
	{
		if(obstacles.empty()) return agent.get_linear_velocity();

		// initialize the list of matrices for calculation
		std::deque<Eigen::Matrix3f> modulation_matrix_list;
		std::deque<Eigen::Matrix3f> orthogonal_basis_list;
		Eigen::ArrayXf distances(obstacles.size());

		// compute all necessary elements to calculation the modulation matrix
		int k = 0;
		for(auto &obs_it : obstacles)
		{
			auto matrices = Modulation::compute_modulation_matrix(agent, *obs_it);
			// store matrices used later
			modulation_matrix_list.push_back(std::get<0>(matrices));
			orthogonal_basis_list.push_back(std::get<1>(matrices));		
			distances(k) = std::get<2>(matrices);
			++k;
		}

		Eigen::ArrayXf weights = Modulation::weight_obstacles(distances, 1.0, 2.0);
		Eigen::Vector3f rel_velocity = Modulation::compute_relative_velocity(agent, obstacles, distances, weights, orthogonal_basis_list);

	}
}