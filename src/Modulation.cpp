#include "DynamicObstacleAvoidance/Modulation.hpp"

namespace Modulation
{
	std::tuple<Eigen::Matrix3d, Eigen::Matrix3d, double> compute_modulation_matrix(const Agent& agent, const Obstacle& obstacle)
	{
		// compute all necessary vectors
		Eigen::Vector3d normal_vector = obstacle.compute_normal_to_agent(agent);
		double distance_to_obstacle = obstacle.compute_distance_to_agent(agent);

		// compute the basis matrices
		auto basis_matrices = compute_basis_matrices(normal_vector, agent, obstacle);
		Eigen::Matrix3d reference_basis = std::get<0>(basis_matrices);
		Eigen::Matrix3d orthogonal_basis = std::get<1>(basis_matrices);

		// compute the diagonal eigenvalues
		Eigen::DiagonalMatrix<double, 3> diagonal_eigenvalues = compute_diagonal_eigenvalues(distance_to_obstacle);

		// compute the modulation matrix
		Eigen::Matrix3d modulation_matrix = obstacle.get_orientation() * reference_basis * diagonal_eigenvalues * reference_basis.inverse() * obstacle.get_orientation().inverse();
		// return all computed elements
		return std::make_tuple(modulation_matrix, orthogonal_basis, distance_to_obstacle);
	}

	Eigen::ArrayXd weight_obstacles(const Eigen::ArrayXd& distances, const double& critical_distance, const double& weight_power)
	{
		Eigen::ArrayXd weights(distances.size());
		Eigen::ArrayXd critical_obstacles(distances.size());

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

	Eigen::DiagonalMatrix<double, 3> compute_diagonal_eigenvalues(const double& distance_to_obstacle, const double& reactivity_factor)
	{
		// check if the point is inside the obstacle
		double delta_eigenvalue = (distance_to_obstacle <= 1) ? 1.0 : 1.0 / std::pow(distance_to_obstacle, 1.0 / reactivity_factor);
		double eigenvalue_reference = 1 - delta_eigenvalue;
		double eigenvalue_tangent = 1 + delta_eigenvalue;
		Eigen::DiagonalMatrix<double, 3> diagonal_eigenvalues(eigenvalue_reference, eigenvalue_tangent, eigenvalue_tangent);
		return diagonal_eigenvalues;
	}

	std::pair<Eigen::Matrix3d, Eigen::Matrix3d> compute_basis_matrices(const Eigen::Vector3d& normal_vector, const Agent& agent, const Obstacle& obstacle)
	{
		Eigen::Vector3d unit_vector = Eigen::Vector3d::UnitZ();
		Eigen::Vector3d tangent_vector = normal_vector.cross(unit_vector);
		
		if(tangent_vector.norm() < 1E-4) 
		{
			unit_vector = Eigen::Vector3d::UnitY();
			tangent_vector = normal_vector.cross(unit_vector);
		}

		Eigen::Vector3d cross_product = tangent_vector.cross(normal_vector);
		Eigen::Matrix3d orthogonal_basis;
		orthogonal_basis << normal_vector, tangent_vector, cross_product;

		Eigen::Vector3d reference_direction = obstacle.get_pose().inverse() * agent.get_position() - obstacle.get_pose().inverse() * obstacle.get_reference_position();
		reference_direction.normalize();

		if(tangent_vector.cross(reference_direction).norm() < 1E-4)
		{
			std::cerr << "Problem here" << std::endl;
		}
		
		Eigen::Vector3d ref_cross_product = tangent_vector.cross(reference_direction);
		Eigen::Matrix3d reference_basis;
		reference_basis << reference_direction, tangent_vector, ref_cross_product;
		return std::make_pair(reference_basis, orthogonal_basis);
	}

	std::pair<Eigen::Vector3d, Eigen::Vector3d> compute_relative_velocities(const Agent& agent, const std::deque<std::unique_ptr<Obstacle> >& obstacles, const Eigen::ArrayXd& distances, const Eigen::ArrayXd& weights, const std::deque<Eigen::Matrix3d>& orthogonal_basis_list)
	{
		Eigen::Vector3d obs_velocity(0,0,0);
		int i = 0;
		for(auto &obs:obstacles)
		{
			Eigen::Vector3d angular_velocity = obs->get_angular_velocity().cross(agent.get_position() - obs->get_position());
			double exp_weight = exp(-(std::max<double>(1.0, distances(i))-1));
			Eigen::Vector3d obs_velocity_tmp = exp_weight * (obs->get_linear_velocity() + angular_velocity);
			obs_velocity_tmp = orthogonal_basis_list[i].transpose() * obs_velocity_tmp;
			obs_velocity_tmp = orthogonal_basis_list[i] * obs_velocity_tmp;
			// sum to get the velocity of all obstackes
			obs_velocity += obs_velocity_tmp * weights(i);
			++i;
		}
		return std::make_pair(agent.get_linear_velocity() - obs_velocity, obs_velocity);
	}

	Eigen::Matrix3d compute_dynamical_system_frame(const Eigen::Vector3d& agent_relative_velocity) 
	{
		Eigen::Vector3d unit_vector = Eigen::Vector3d::UnitZ();
		Eigen::Vector3d cross_product = agent_relative_velocity.cross(unit_vector);
		if(cross_product.norm() < 1E-4) 
		{
			cross_product = Eigen::Vector3d::UnitY();
			cross_product = agent_relative_velocity.cross(unit_vector);
		}
		Eigen::Matrix3d orthogonal_matrix;
		orthogonal_matrix << agent_relative_velocity, cross_product, cross_product.cross(agent_relative_velocity);
		return orthogonal_matrix;
	}

	double safe_acos(double x)
  	{
  		if (x < -1.0) x = -1.0 ;
  		else if (x > 1.0) x = 1.0 ;
  		return acos (x) ;
  	}

	Eigen::Vector3d modulate_velocity(const Agent& agent, const std::deque<std::unique_ptr<Obstacle> >& obstacles, const double& critical_distance, const double& weight_power)
	{
		if(obstacles.empty()) return agent.get_linear_velocity();

		// no modulation if no obstacle in sight
		if(!agent.get_envelope().is_intersecting(obstacles)) return agent.get_linear_velocity();

		// initialize the list of matrices for calculation
		std::deque<Eigen::Matrix3d> modulation_matrix_list;
		std::deque<Eigen::Matrix3d> orthogonal_basis_list;
		Eigen::ArrayXd distances(obstacles.size());

		// compute all necessary elements to calculation the modulation matrix
		int i = 0;
		for(auto &obs_it : obstacles)
		{
			if(obs_it->get_type() == "Aggregate")
			{
				const Obstacle& obstacle = static_cast<Aggregate*>(obs_it.get())->get_active_obstacle(agent);
				auto matrices = Modulation::compute_modulation_matrix(agent, obstacle);
				// store matrices used later
				modulation_matrix_list.push_back(std::get<0>(matrices));
				orthogonal_basis_list.push_back(std::get<1>(matrices));		
				distances(i) = std::get<2>(matrices);
			}
			else
			{
				auto matrices = Modulation::compute_modulation_matrix(agent, *obs_it);
				// store matrices used later
				modulation_matrix_list.push_back(std::get<0>(matrices));
				orthogonal_basis_list.push_back(std::get<1>(matrices));		
				distances(i) = std::get<2>(matrices);
			}
			++i;
		}

		Eigen::ArrayXd weights = Modulation::weight_obstacles(distances, critical_distance, weight_power);
		auto velocities = Modulation::compute_relative_velocities(agent, obstacles, distances, weights, orthogonal_basis_list);
		Eigen::Vector3d agent_relative_velocity = std::get<0>(velocities);
		Eigen::Vector3d obstacles_relative_velocity = std::get<1>(velocities);

		Eigen::Vector3d normalized_relative_velocity = agent_relative_velocity.normalized();
		Eigen::Matrix3d ds_frame = compute_dynamical_system_frame(normalized_relative_velocity);
		Eigen::Matrix3d transposed_ds_frame = ds_frame.transpose();

		i = 0;
		Eigen::Vector2d sum_modulated_velocity_angle_space(0,0);
		double velocity_magnitude = 0.0;
		for(auto m:modulation_matrix_list)
		{
			Eigen::Vector3d modulated_velocity = m * agent_relative_velocity;
			velocity_magnitude += modulated_velocity.norm() * weights(i);
			Eigen::Vector3d normalized_modulated_velocity = modulated_velocity.normalized();
			Eigen::Vector3d transposed_normalized_modulated_velocity = transposed_ds_frame * normalized_modulated_velocity;

			Eigen::Vector2d modulated_velocity_angle_space = transposed_normalized_modulated_velocity.tail<2>();
			modulated_velocity_angle_space.normalize();
			
			double cos_angle = (normalized_modulated_velocity.array() * normalized_relative_velocity.array()).sum();
			modulated_velocity_angle_space *= Modulation::safe_acos(cos_angle);
			modulated_velocity_angle_space *= weights(i);
			sum_modulated_velocity_angle_space += modulated_velocity_angle_space;

			++i;
		}

		double norm_modulated_velocity_angle_space = sum_modulated_velocity_angle_space.norm();
		Eigen::Vector3d reconstructed_velocity;

		if(norm_modulated_velocity_angle_space == 0)
		{
			reconstructed_velocity << 1, sum_modulated_velocity_angle_space;
		}
		else
		{
			reconstructed_velocity << cos(norm_modulated_velocity_angle_space), (sin(norm_modulated_velocity_angle_space) / norm_modulated_velocity_angle_space) * sum_modulated_velocity_angle_space;
		}		

		Eigen::Vector3d modulated_velocity = ds_frame * reconstructed_velocity;
		modulated_velocity *= velocity_magnitude;
		modulated_velocity += obstacles_relative_velocity;
		return modulated_velocity;
	}
}