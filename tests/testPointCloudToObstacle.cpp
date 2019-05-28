#include "DynamicObstacleAvoidance/Obstacle/PointCloudToObstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Utils/Plotting/PlottingTools.hpp"
#include "DynamicObstacleAvoidance/Agent.hpp"
#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <deque>
#include <stdlib.h> 

TEST(TestFitEllipsoid, PositiveNos)
{
	Eigen::MatrixXd surface_points(3,10);
	surface_points << 0.68026725, 3.80951844, -1.6613724, -0.25730725, 0.69801745, -5.8991695, -5.46727359, -5.68929574, -4.95414465, -5.78085261,
					  -0.01634235, 0.79848348, -0.57766695, -0.15555173, 0.17456129, 4.01799145, 3.08517251, 3.23357203, 2.85638922, 2.40213298,
					  0,0,0,0,0,0,0,0,0,0;

	PointCloudToObstacle pco(0.5, 2);
	auto ellipse_list = pco.fit_ellipsoids_on_cluster(surface_points, 2);
	ASSERT_EQ(ellipse_list.size(), 2);

	std::deque<Ellipsoid> plot_list;

	for(auto &e:ellipse_list)
	{
		std::cerr << "position: " << e->get_position() << std::endl;
		std::cerr << "rotation: " << e->get_orientation().w() << ", " << e->get_orientation().x() << ", " << e->get_orientation().y() << ", " << e->get_orientation().z() << std::endl;
		std::cerr << "axis: " << e->get_axis_lengths() << std::endl;
		std::cerr << "--------" << std::endl;
		plot_list.push_back(*e);
	}

	//PlottingTools::plot_ellipsoids2D(plot_list);
}

TEST(TestClustering, PositiveNos)
{
	Eigen::MatrixXd surface_points(3,10);
	surface_points << 0.68026725, 3.80951844, -1.6613724, -0.25730725, 0.69801745, -5.8991695, -5.46727359, -5.68929574, -4.95414465, -5.78085261,
					  -0.01634235, 0.79848348, -0.57766695, -0.15555173, 0.17456129, 4.01799145, 3.08517251, 3.23357203, 2.85638922, 2.40213298,
					  0,0,0,0,0,0,0,0,0,0;

	PointCloudToObstacle pco(0.5, 2);
	std::deque<Eigen::MatrixXd> clusters = pco.cluster_surface_points(surface_points);

	ASSERT_EQ(clusters.size(), 2);

	for(Eigen::MatrixXd c:clusters) 
	{
		std::cerr << c << std::endl;
		std::cerr << "------------" << std::endl;
	}

	//PlottingTools::plot_clusters(clusters);
}

Eigen::VectorXd normal_random_variable(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covar)
{
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
    Eigen::MatrixXd transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    std::mt19937 gen{ std::random_device{}() };
    std::normal_distribution<> dist;
    return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](auto x) { return dist(gen); });
};

TEST(FitObstacles, PositiveNos)
{
	int nb_clusters = 10;
	int nb_points_by_cluster = 100;
	Eigen::MatrixXd surface_points(3, nb_clusters*nb_points_by_cluster);

	std::deque<Eigen::MatrixXd> clusters(nb_clusters);
	for(int i=0; i<nb_clusters; ++i)
	{
		Eigen::VectorXd center = Eigen::VectorXd::Random(3) * 20;
		center(2) = 0;
		double sigma_xx = rand() % 3;
		double sigma_yy = rand() % 3;;
		double sigma_xy = rand() % 3;;
		Eigen::MatrixXd covar(3, 3);
		covar << sigma_xx, sigma_xy, 0,
        		sigma_xy, sigma_yy, 0,
        		0, 0, 1;

		Eigen::MatrixXd cluster_points(3, nb_points_by_cluster);
		for(int j=0; j<nb_points_by_cluster; ++j)
		{
			cluster_points.col(j) = normal_random_variable(center, covar);
			surface_points.col(nb_points_by_cluster*i + j) = cluster_points.col(j);
		}
		clusters[i] = cluster_points;
	}

	// cluster it
	PointCloudToObstacle pco(1, 10);

	/*auto obstacle_list = pco.fit_obstacles(surface_points);
	std::deque<Ellipsoid> ellipse_list;
	for(auto &o:obstacle_list)
	{
		Ellipsoid e = *static_cast<Ellipsoid*>(o.get());
		std::cerr << "position: " << e.get_position() << std::endl;
		std::cerr << "rotation: " << e.get_orientation().w() << ", " << e.get_orientation().x() << ", " << e.get_orientation().y() << ", " << e.get_orientation().z() << std::endl;
		std::cerr << "axis: " << e.get_axis_lengths() << std::endl;
		std::cerr << "--------" << std::endl;
		ellipse_list.push_back(e);
	}*/
	//PlottingTools::plot_fitted_clusters(clusters, ellipse_list);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}