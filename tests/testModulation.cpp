#include "DynamicObstacleAvoidance/Modulation.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid2D.hpp"
#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>


TEST(WeightObstacleUnderCriticalDistance, PositiveNos)
{
	Eigen::ArrayXf distances(5);
	distances << 0, 1, 2, 3, 4;
	float critical_distance = 1.0;
	float weight_power = 2;
	Eigen::ArrayXf weights = Modulation::weight_obstacles(distances, critical_distance, weight_power);

	Eigen::ArrayXf true_values(5);
	true_values << 0.5, 0.5, 0. , 0. , 0. ;

	std::cerr << "weights: " << weights << std::endl;
	std::cerr << "-------" << std::endl;
	std::cerr << "truth: " << true_values << std::endl;

	for(int i=0; i<weights.size(); ++i) ASSERT_NEAR(weights(i), true_values(i), 0.00001);
}

TEST(WeightObstacleFirstEqualCriticalDistance, PositiveNos)
{
	Eigen::ArrayXf distances(5);
	distances << 1, 2, 3, 4, 5;
	float critical_distance = 1.0;
	float weight_power = 2;
	Eigen::ArrayXf weights = Modulation::weight_obstacles(distances, critical_distance, weight_power);

	Eigen::ArrayXf true_values(5);
	true_values << 1., 0., 0., 0., 0.;

	std::cerr << "weights: " << weights << std::endl;
	std::cerr << "-------" << std::endl;
	std::cerr << "truth: " << true_values << std::endl;

	for(int i=0; i<weights.size(); ++i) ASSERT_NEAR(weights(i), true_values(i), 0.00001);
}

TEST(WeightObstacleAboveCriticalDistance, PositiveNos)
{
	Eigen::ArrayXf distances(5);
	distances << 2, 3, 4, 5, 6;
	float critical_distance = 1.0;
	float weight_power = 2;
	Eigen::ArrayXf weights = Modulation::weight_obstacles(distances, critical_distance, weight_power);

	Eigen::ArrayXf true_values(5);
	true_values << 0.6832416 , 0.1708104 , 0.07591573, 0.0427026 , 0.02732966;

	std::cerr << "weights: " << weights << std::endl;
	std::cerr << "-------" << std::endl;
	std::cerr << "truth: " << true_values << std::endl;

	for(int i=0; i<weights.size(); ++i) ASSERT_NEAR(weights(i), true_values(i), 0.00001);
}

TEST(ComputeBasisMatrices, PositiveNos)
{
	Eigen::Vector3f position(2, 1, 0);
	State s(position);
	Ellipsoid2D e(s);

	Eigen::Vector3f agent_position(1, 0, 0);

	Eigen::Vector3f normal = e.compute_normal_to_external_point(agent_position);
	auto basis_matrices = Modulation::compute_basis_matrices(normal, agent_position, e.get_reference_position());

	Eigen::Matrix3f reference_basis = std::get<0>(basis_matrices);
	Eigen::Matrix3f orthogonal_basis = std::get<1>(basis_matrices);

	Eigen::Matrix3f reference_basis_truth;
	reference_basis_truth << -0.70710678, -0.70710678, 0,
							 -0.70710678, 0.70710678, 0,
							 0, 0, 1;
	Eigen::Matrix3f orthogonal_basis_truth;
	orthogonal_basis_truth << -0.70710678, -0.70710678, 0,
							  -0.70710678, 0.70710678, 0,
							  0, 0, 1;

	std::cerr << "reference_basis: " << std::endl;
	std::cerr << reference_basis << std::endl;
	std::cerr << "--------------" << std::endl;
	std::cerr << "reference_basis_truth: " << std::endl;
	std::cerr << reference_basis_truth << std::endl;

	std::cerr << "orthogonal_basis: " << std::endl;
	std::cerr << orthogonal_basis << std::endl;
	std::cerr << "--------------" << std::endl;
	std::cerr << "orthogonal_basis_truth: " << std::endl;
	std::cerr << orthogonal_basis_truth << std::endl;

	for(int i=0; i<reference_basis.rows(); ++i)
	{
		for(int j=0; j<reference_basis.cols(); ++j) ASSERT_NEAR(reference_basis(i, j), reference_basis_truth(i, j), 0.0001);
	} 
}

TEST(ComputeBasisMatricesWithRotation, PositiveNos)
{
	Eigen::Vector3f position(2, 1, 0);
	Eigen::Quaternionf orientation(Eigen::AngleAxisf(0.3*M_PI, Eigen::Vector3f::UnitZ()));
	Ellipsoid2D e(State(position, orientation));

	Eigen::Vector3f agent_position(1, 0, 0);

	Eigen::Vector3f normal = e.compute_normal_to_external_point(agent_position);
	auto basis_matrices = Modulation::compute_basis_matrices(normal, agent_position, e.get_reference_position());
	Eigen::Matrix3f orthogonal_basis = std::get<1>(basis_matrices);

	Eigen::Matrix3f orthogonal_basis_truth;
	orthogonal_basis_truth << -0.98768834, 0.15643447, 0,
       						  0.15643447, 0.98768834, 0,
							  0, 0, 1;

	std::cerr << "orthogonal_basis: " << std::endl;
	std::cerr << orthogonal_basis << std::endl;
	std::cerr << "--------------" << std::endl;
	std::cerr << "orthogonal_basis_truth: " << std::endl;
	std::cerr << orthogonal_basis_truth << std::endl;

	for(int i=0; i<orthogonal_basis.rows(); ++i)
	{
		for(int j=0; j<orthogonal_basis.cols(); ++j) ASSERT_NEAR(orthogonal_basis(i, j), orthogonal_basis_truth(i, j), 0.0001);
	} 
}

TEST(ComputeDiagonalEigenvalues, PositiveNos)
{
	Eigen::Vector3f position(2, 1, 0);
	State s(position);
	Ellipsoid2D e(s);

	Eigen::Vector3f agent_position(1, 0, 0);

	float distance = e.compute_distance_to_external_point(agent_position);
	Eigen::DiagonalMatrix<float, 3> eigenvalues = Modulation::compute_diagonal_eigenvalues(distance);

	Eigen::DiagonalMatrix<float, 3> eigenvalues_truth(0.5, 1.5, 1.5);

	std::cerr << "eigenvalues: " << std::endl;
	std::cerr << eigenvalues.diagonal() << std::endl;
	std::cerr << "--------------" << std::endl;
	std::cerr << "eigenvalues truth: " << std::endl;
	std::cerr << eigenvalues_truth.diagonal() << std::endl;

	for(int i=0; i<eigenvalues.diagonal().size(); ++i) ASSERT_NEAR(eigenvalues.diagonal()(i), eigenvalues_truth.diagonal()(i), 0.0001);
}

TEST(ComputeModulationMatrix, PositiveNos)
{
	Eigen::Vector3f position(2, 1, 0);
	State s(position);
	Ellipsoid2D e(s);

	Eigen::Vector3f agent_position(1,0,0);
	State agent_state(agent_position);
	Agent agent(agent_state);

	auto matrices = Modulation::compute_modulation_matrix(agent, e);
	Eigen::Matrix3f modulation_matrix = std::get<0>(matrices);
	Eigen::Matrix3f	orthogonal_basis = std::get<1>(matrices);
	float distance = std::get<2>(matrices);

	std::cerr << "modulation matrix" << std::endl;
	std::cerr << modulation_matrix << std::endl;
	std::cerr << "orthogonal basis" << std::endl;
	std::cerr << orthogonal_basis << std::endl;
	std::cerr << "distance" << std::endl;
	std::cerr << distance << std::endl;
	std::cerr << "-----------" << std::endl;

	Eigen::Matrix3f modulation_matrix_truth;
	modulation_matrix_truth << 1, -0.5, 0,
							   -0.5, 1, 0,
							   0, 0, 1.5;

	
	Eigen::Matrix3f orthogonal_basis_truth;
	orthogonal_basis_truth << -0.70710678, -0.70710678, 0,
							  -0.70710678, 0.70710678, 0,
							  0, 0, 1;
	float distance_truth = 2.0;

	for(int i=0; i<modulation_matrix.rows(); ++i)
	{
		for(int j=0; j<modulation_matrix.cols(); ++j) ASSERT_NEAR(modulation_matrix(i, j), modulation_matrix_truth(i, j), 0.0001);
	}
	for(int i=0; i<orthogonal_basis.rows(); ++i)
	{
		for(int j=0; j<orthogonal_basis.cols(); ++j) ASSERT_NEAR(orthogonal_basis(i, j), orthogonal_basis_truth(i, j), 0.0001);
	}
	ASSERT_NEAR(distance, distance_truth, 0.0001);
}

TEST(ComputeRelativeVelocities, PositiveNos)
{
	Eigen::Vector3f position_o1(2, 1, 0);
	Eigen::Vector3f position_o2(0, 0, 0);
	Eigen::Vector3f position_o3(0.9, 0, 0);
	Eigen::Quaternionf orientation(1,0,0,0);

	auto ptrE1 = std::make_unique<Ellipsoid2D>(State(position_o1, orientation));
	auto ptrE2 = std::make_unique<Ellipsoid2D>(State(position_o2, orientation));
	ptrE2->set_linear_velocity(Eigen::Vector3f(-0.5,-0.5,0));
	auto ptrE3 = std::make_unique<Ellipsoid2D>(State(position_o3, orientation));

	std::deque<std::unique_ptr<Obstacle> > obstacle_list;
	obstacle_list.push_back(std::move(ptrE1));
	obstacle_list.push_back(std::move(ptrE2));
	obstacle_list.push_back(std::move(ptrE3));

	Eigen::Vector3f agent_position(1,0,0);
	State agent_state(agent_position);
	Agent agent(agent_state);
	agent.set_linear_velocity(Eigen::Vector3f(0.5,0.5,0));

	std::deque<Eigen::Matrix3f> orthogonal_basis_list;
	Eigen::ArrayXf distances(obstacle_list.size());

	// compute all necessary elements to calculation the modulation matrix
	int k = 0;
	for(auto &obs_it : obstacle_list)
	{
		auto matrices = Modulation::compute_modulation_matrix(agent, *obs_it);
		orthogonal_basis_list.push_back(std::get<1>(matrices));	
		distances(k) = std::get<2>(matrices);
		++k;
	}

	Eigen::ArrayXf weights = Modulation::weight_obstacles(distances, 1.0, 2.0);
	auto velocities = Modulation::compute_relative_velocities(agent, obstacle_list, distances, weights, orthogonal_basis_list);
	Eigen::Vector3f agent_relative_velocity = std::get<0>(velocities);
	Eigen::Vector3f obstacles_relative_velocity = std::get<1>(velocities);
	
	Eigen::Vector3f rel_velocity_truth(0.75, 0.75, 0);
	Eigen::Vector3f obs_velocity_truth(-0.25, -0.25, 0);

	std::cerr << "agent relative velocity: " << std::endl;
	std::cerr << agent_relative_velocity << std::endl;
	std::cerr << "--------------" << std::endl;
	std::cerr << "rel_velocity truth: " << std::endl;
	std::cerr << rel_velocity_truth << std::endl;

	std::cerr << "obstacle relative velocity: " << std::endl;
	std::cerr << obstacles_relative_velocity << std::endl;
	std::cerr << "--------------" << std::endl;
	std::cerr << "rel_velocity truth: " << std::endl;
	std::cerr << obs_velocity_truth << std::endl;

	for(int i=0; i<agent_relative_velocity.size(); ++i) ASSERT_NEAR(agent_relative_velocity(i), rel_velocity_truth(i), 0.0001);
	for(int i=0; i<obstacles_relative_velocity.size(); ++i) ASSERT_NEAR(obstacles_relative_velocity(i), obs_velocity_truth(i), 0.0001);
}

TEST(ModulateVelocity, PositiveNos)
{
	Eigen::Vector3f position_o1(2, 1, 0);
	Eigen::Vector3f position_o2(0, 0, 0);
	Eigen::Vector3f position_o3(0.9, 0, 0);
	Eigen::Quaternionf orientation(1,0,0,0);

	auto ptrE1 = std::make_unique<Ellipsoid2D>(State(position_o1, orientation));
	auto ptrE2 = std::make_unique<Ellipsoid2D>(State(position_o2, orientation));
	ptrE2->set_linear_velocity(Eigen::Vector3f(-0.5,-0.5,0));
	auto ptrE3 = std::make_unique<Ellipsoid2D>(State(position_o3, orientation));

	std::deque<std::unique_ptr<Obstacle> > obstacle_list;
	obstacle_list.push_back(std::move(ptrE1));
	obstacle_list.push_back(std::move(ptrE2));
	obstacle_list.push_back(std::move(ptrE3));

	Eigen::Vector3f agent_position(1,0,0);
	State agent_state(agent_position);
	Agent agent(agent_state);
	agent.set_linear_velocity(Eigen::Vector3f(0.5,0.5,0));

	Eigen::Vector3f modulated_velocity = Modulation::modulate_velocity(agent, obstacle_list);
	Eigen::Vector3f modulated_velocity_truth(-0.25,1.25,0.0);

	std::cerr << "modulated velocity: " << std::endl;
	std::cerr << modulated_velocity << std::endl;
	std::cerr << "--------------" << std::endl;
	std::cerr << "modulated_velocity truth: " << std::endl;
	std::cerr << modulated_velocity_truth << std::endl;

	for(int i=0; i<modulated_velocity.size(); ++i) ASSERT_NEAR(modulated_velocity(i), modulated_velocity_truth(i), 0.0001);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}