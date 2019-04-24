#include "DynamicObstacleAvoidance/Modulation.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid2D.hpp"
#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>

TEST(WeightObstacleUnderCriticalDistance, PositiveNos)
{
	Eigen::ArrayXf distances(5);
	distances << 0, 1, 2, 3, 4;
	double critical_distance = 1.0;
	double weight_power = 2;
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
	double critical_distance = 1.0;
	double weight_power = 2;
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
	double critical_distance = 1.0;
	double weight_power = 2;
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
	Eigen::Vector3f position;
	position << 2, 1, 0;

	Eigen::Vector4f orientation;
	orientation << 0, 0, 0, 1;

	Ellipsoid2D e(position, orientation);

	Eigen::Vector3f agent_position;
	agent_position << 1, 0, 0;

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
		for(int j=0; i<reference_basis.cols(); ++i) ASSERT_NEAR(reference_basis(i, j), reference_basis_truth(i, j), 0.00001);
	} 
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}