#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Agent.hpp"
#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>

TEST(ComputeDistanceToExternalPointNonNullPosition, PositiveNos)
{
	Eigen::Vector3d position(2, 1, 0);
	State s(position);
	Ellipsoid e(s);

	Eigen::Vector3d agent_position(1, 0, 0);
	State state(agent_position);
	Agent agent(state);

	std::cerr << "Computing ditance" << std::endl;
	double distance = e.compute_distance_to_agent(agent);
	double truth = 2.0;
	std::cerr << "distance = " << distance << ", truth = " << truth << std::endl;

	ASSERT_NEAR(distance, truth, 0.01);
}

TEST(ComputeDistanceToExternalPointNullPosition, PositiveNos)
{
	Eigen::Vector3d position(0, 0, 0);
	State s(position);
	Ellipsoid e(s);

	Eigen::Vector3d agent_position(1, 0, 0);
	State state(agent_position);
	Agent agent(state);

	std::cerr << "Computing ditance" << std::endl;
	double distance = e.compute_distance_to_agent(agent);
	double truth = 1.0;
	std::cerr << "distance = " << distance << ", truth = " << truth << std::endl;

	ASSERT_NEAR(distance, truth, 0.01);
}

TEST(ComputeDistanceToExternalPointCloseToObstacle, PositiveNos)
{
	Eigen::Vector3d position(0.9, 0, 0);
	State s(position);
	Ellipsoid e(s);

	Eigen::Vector3d agent_position(1, 0, 0);
	State state(agent_position);
	Agent agent(state);

	std::cerr << "Computing ditance" << std::endl;
	double distance = e.compute_distance_to_agent(agent);
	double truth = 0.01;
	std::cerr << "distance = " << distance << ", truth = " << truth << std::endl;

	ASSERT_NEAR(distance, truth, 0.01);
}

TEST(ComputeNormalToExternalPointNonNullPosition, PositiveNos)
{
	Eigen::Vector3d position(2, 1, 0);
	State s(position);
	Ellipsoid e(s);

	Eigen::Vector3d agent_position(1, 0, 0);
	State state(agent_position);
	Agent agent(state);

	std::cerr << "Computing normal" << std::endl;
	Eigen::Vector3d normal = e.compute_normal_to_agent(agent);
	Eigen::Vector3d truth(-0.70710678, -0.70710678, 0.0);
	std::cerr << "normal = " << normal << ", truth = " << truth << std::endl;

	for(int i=0; i<normal.size(); ++i) ASSERT_NEAR(normal(i), truth(i), 0.01);
}

TEST(ComputeNormalToExternalPointNullPosition, PositiveNos)
{
	Eigen::Vector3d position(0, 0, 0);
	State s(position);
	Ellipsoid e(s);

	Eigen::Vector3d agent_position(1, 0, 0);
	State state(agent_position);
	Agent agent(state);

	std::cerr << "Computing normal" << std::endl;
	Eigen::Vector3d normal = e.compute_normal_to_agent(agent);
	Eigen::Vector3d truth(1.0, 0.0, 0.0);
	std::cerr << "normal = " << normal << ", truth = " << truth << std::endl;

	for(int i=0; i<normal.size(); ++i) ASSERT_NEAR(normal(i), truth(i), 0.01);
}

TEST(ComputeNormalToExternalPointCloseToObstacle, PositiveNos)
{
	Eigen::Vector3d position(0.9, 0, 0);
	State s(position);
	Ellipsoid e(s);

	Eigen::Vector3d agent_position(1, 0, 0);
	State state(agent_position);
	Agent agent(state);

	std::cerr << "Computing normal" << std::endl;
	Eigen::Vector3d normal = e.compute_normal_to_agent(agent);
	Eigen::Vector3d truth(1.0, 0.0, 0.0);
	std::cerr << "normal = " << normal << ", truth = " << truth << std::endl;

	for(int i=0; i<normal.size(); ++i) ASSERT_NEAR(normal(i), truth(i), 0.01);
}

TEST(IsInside, PositiveNos)
{
	Ellipsoid e1;
	Ellipsoid e2;
	e2.set_axis_lengths(Eigen::Vector3d(2,2,0));

	ASSERT_EQ(e1.is_intersecting(e2), true);
	ASSERT_EQ(e2.is_intersecting(e1), true);
}


TEST(IsInteresecting, PositiveNos)
{
	Ellipsoid e1;
	Eigen::Vector3d position_o2(1,1,0);
	Eigen::Quaterniond orientation_o2(Eigen::AngleAxisd(0.75, Eigen::Vector3d::UnitZ()));
	Ellipsoid e2(State(position_o2, orientation_o2));

	ASSERT_EQ(e1.is_intersecting(e2), true);
	ASSERT_EQ(e2.is_intersecting(e1), true);
}

TEST(IsNotInteresecting, PositiveNos)
{
	Ellipsoid e1;
	Eigen::Vector3d position_o2(4,4,0);
	Eigen::Quaterniond orientation_o2(Eigen::AngleAxisd(0.75, Eigen::Vector3d::UnitZ()));
	Ellipsoid e2(State(position_o2, orientation_o2));

	ASSERT_EQ(e1.is_intersecting(e2), false);
	ASSERT_EQ(e2.is_intersecting(e1), false);
}

TEST(IsInteresectingBothRotations, PositiveNos)
{
	Eigen::Vector3d position_o5(3.7, 4.7, 0);
	Eigen::Vector3d position_o6(4.6, 4.7, 0);
	Eigen::Quaterniond orientation_o5(Eigen::AngleAxisd(0.75, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o6(Eigen::AngleAxisd(-0.75, Eigen::Vector3d::UnitZ()));

	Ellipsoid e1(State(position_o5, orientation_o5));
	Ellipsoid e2(State(position_o6, orientation_o6));
	e1.set_axis_lengths(Eigen::Array3d(0.75, 0.2, 0));
	e2.set_axis_lengths(Eigen::Array3d(0.75, 0.2, 0));

	ASSERT_EQ(e1.is_intersecting(e2), true);
	ASSERT_EQ(e2.is_intersecting(e1), true);
}

TEST(IsNotInteresectingBothRotations, PositiveNos)
{
	Eigen::Vector3d position_o1(3, 0.62, 0);
	Eigen::Vector3d position_o6(4.6, 4.7, 0);

	Eigen::Quaterniond orientation_o1(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond orientation_o6(Eigen::AngleAxisd(-0.75, Eigen::Vector3d::UnitZ()));

	Ellipsoid e1(State(position_o1, orientation_o1));
	Ellipsoid e2(State(position_o6, orientation_o6));

	e1.set_axis_lengths(Eigen::Array3d(0.05, 0.7, 0));
	e2.set_axis_lengths(Eigen::Array3d(0.75, 0.2, 0));

	ASSERT_EQ(e1.is_intersecting(e2), false);
	ASSERT_EQ(e2.is_intersecting(e1), false);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}