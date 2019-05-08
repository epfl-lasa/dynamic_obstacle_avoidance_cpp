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
		Eigen::DiagonalMatrix<float, 3> diagonal_eigenvalues = compute_diagonal_eigenvalues(distance_to_obstacle);

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

	Eigen::DiagonalMatrix<float, 3> compute_diagonal_eigenvalues(const float& distance_to_obstacle, const float& reactivity_factor)
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

	std::pair<Eigen::Vector3f, Eigen::Vector3f> compute_relative_velocities(const Agent& agent, const std::deque<std::unique_ptr<Obstacle> >& obstacles, const Eigen::ArrayXf& distances, const Eigen::ArrayXf& weights, const std::deque<Eigen::Matrix3f>& orthogonal_basis_list)
	{
		Eigen::Vector3f obs_velocity(0,0,0);
		int i = 0;
		for(auto &obs:obstacles)
		{
			Eigen::Vector3f angular_velocity = obs->get_angular_velocity().cross(agent.get_position() - obs->get_position());
			float exp_weight = exp(-(std::max<float>(1.0, distances(i))-1));
			Eigen::Vector3f obs_velocity_tmp = exp_weight * (obs->get_linear_velocity() + angular_velocity);
			obs_velocity_tmp = orthogonal_basis_list[i].transpose() * obs_velocity_tmp;
			obs_velocity_tmp = orthogonal_basis_list[i] * obs_velocity_tmp;
			// sum to get the velocity of all obstackes
			obs_velocity += obs_velocity_tmp * weights(i);
			++i;
		}
		return std::make_pair(agent.get_linear_velocity() - obs_velocity, obs_velocity);
	}

	Eigen::Matrix3f compute_dynamical_system_frame(const Eigen::Vector3f& agent_relative_velocity) 
	{
		Eigen::Vector3f unit_vector = Eigen::Vector3f::UnitZ();
		Eigen::Vector3f cross_product = agent_relative_velocity.cross(unit_vector);
		if(cross_product.norm() == 0) 
		{
			cross_product = Eigen::Vector3f::UnitY();
			cross_product = agent_relative_velocity.cross(unit_vector);
		}
		Eigen::Matrix3f orthogonal_matrix;
		orthogonal_matrix << agent_relative_velocity, cross_product, cross_product.cross(agent_relative_velocity);
		return orthogonal_matrix;
	}

	double safe_acos(double x)
  	{
  		if (x < -1.0) x = -1.0 ;
  		else if (x > 1.0) x = 1.0 ;
  		return acos (x) ;
  	}

	Eigen::Vector3f modulate_velocity(const Agent& agent, const std::deque<std::unique_ptr<Obstacle> >& obstacles, const float& critical_distance, const float& weight_power)
	{
		if(obstacles.empty()) return agent.get_linear_velocity();

		// initialize the list of matrices for calculation
		std::deque<Eigen::Matrix3f> modulation_matrix_list;
		std::deque<Eigen::Matrix3f> orthogonal_basis_list;
		Eigen::ArrayXf distances(obstacles.size());

		// compute all necessary elements to calculation the modulation matrix
		int i = 0;
		for(auto &obs_it : obstacles)
		{
			auto matrices = Modulation::compute_modulation_matrix(agent, *obs_it);
			// store matrices used later
			modulation_matrix_list.push_back(std::get<0>(matrices));
			orthogonal_basis_list.push_back(std::get<1>(matrices));		
			distances(i) = std::get<2>(matrices);
			++i;
		}

		Eigen::ArrayXf weights = Modulation::weight_obstacles(distances, critical_distance, weight_power);
		auto velocities = Modulation::compute_relative_velocities(agent, obstacles, distances, weights, orthogonal_basis_list);
		Eigen::Vector3f agent_relative_velocity = std::get<0>(velocities);
		Eigen::Vector3f obstacles_relative_velocity = std::get<1>(velocities);

		Eigen::Vector3f normalized_relative_velocity = agent_relative_velocity.normalized();
		Eigen::Matrix3f ds_frame = compute_dynamical_system_frame(normalized_relative_velocity);
		Eigen::Matrix3f transposed_ds_frame = ds_frame.transpose();

		i = 0;
		Eigen::Vector2f sum_modulated_velocity_angle_space(0,0);
		float velocity_magnitude = 0.0;
		for(auto m:modulation_matrix_list)
		{
			Eigen::Vector3f modulated_velocity = m * agent_relative_velocity;
			velocity_magnitude += modulated_velocity.norm() * weights(i);
			Eigen::Vector3f normalized_modulated_velocity = modulated_velocity.normalized();
			Eigen::Vector3f transposed_normalized_modulated_velocity = transposed_ds_frame * normalized_modulated_velocity;

			Eigen::Vector2f modulated_velocity_angle_space = transposed_normalized_modulated_velocity.tail<2>();
			modulated_velocity_angle_space.normalize();
			
			float cos_angle = (normalized_modulated_velocity.array() * normalized_relative_velocity.array()).sum();
			modulated_velocity_angle_space *= Modulation::safe_acos(cos_angle);
			modulated_velocity_angle_space *= weights(i);
			sum_modulated_velocity_angle_space += modulated_velocity_angle_space;

			++i;
		}

		float norm_modulated_velocity_angle_space = sum_modulated_velocity_angle_space.norm();
		Eigen::Vector3f reconstructed_velocity;

		if(norm_modulated_velocity_angle_space == 0)
		{
			reconstructed_velocity << 1, sum_modulated_velocity_angle_space;
		}
		else
		{
			reconstructed_velocity << cos(norm_modulated_velocity_angle_space), (sin(norm_modulated_velocity_angle_space) / norm_modulated_velocity_angle_space) * sum_modulated_velocity_angle_space;
		}		

		Eigen::Vector3f modulated_velocity = ds_frame * reconstructed_velocity;
		modulated_velocity *= velocity_magnitude;
		modulated_velocity += obstacles_relative_velocity;
		return modulated_velocity;
	}
}