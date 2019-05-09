#include "DynamicObstacleAvoidance/Obstacle/LaserScan2D.hpp"

LaserScan2D::LaserScan2D(const Eigen::ArrayXf& distances, const float& starting_angle, const float& delta_angle, const float& safety_margin, const float& z_coordinate)
{}

LaserScan2D::~LaserScan2D()
{}

bool LaserScan2D::learn_gmm_on_points(const Eigen::MatrixXd& surface_points, const int& nb_gaussians)
{
	// convert eigen to armadillo matrix using advanced initialization
	arma::mat data = arma::mat(surface_points.data(), surface_points.rows(), surface_points.cols());

	// learn gmm model on the surface points with default parameters for km_iter, em_iter and var_floor
	bool sucess = this->gmm_model.learn(data, nb_gaussians, arma::eucl_dist, arma::random_subset, 10, 5, 1e-10, false)

}

Eigen::Vector3f LaserScan2D::compute_normal_to_external_point(const Eigen::Vector3f& external_point) const
{}

float LaserScan2D::compute_distance_to_external_point(const Eigen::Vector3f& external_point) const
{}