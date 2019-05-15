#include "DynamicObstacleAvoidance/Obstacle/PointCloudToObstacle.hpp"

PointCloudToObstacle::PointCloudToObstacle(const double& safety_margin)
{}

PointCloudToObstacle::~PointCloudToObstacle()
{}

arma::gmm_full PointCloudToObstacle::learn_gmm_on_points(const Eigen::MatrixXd& surface_points, const int& nb_gaussians)
{
	// convert eigen to armadillo matrix using advanced initialization
	arma::mat data = arma::mat(surface_points.data(), surface_points.rows(), surface_points.cols());

	// learn gmm model on the surface points with default parameters for km_iter, em_iter and var_floor
	arma::gmm_full gmm_model;
	bool success = gmm_model.learn(data, nb_gaussians, arma::eucl_dist, arma::random_subset, 10, 5, 1e-10, false);
	return gmm_model;
}

Eigen::MatrixXd PointCloudToObstacle::from_laser_scan_to_point_cloud(const Eigen::ArrayXd& distances, const double& starting_angle, const double& delta_angle, const double& z_coordinate) const
{

}

std::pair<arma::gmm_full, double> PointCloudToObstacle::compute_bic(const arma::mat& data, const int& nb_gaussians)
{
	int nb_dimensions = data.n_rows;
	int nb_samples = data.n_cols;

	// learn gmm model on the surface points with default parameters for km_iter, em_iter and var_floor
	arma::gmm_full gmm_model;
	bool success = gmm_model.learn(data, nb_gaussians, arma::eucl_dist, arma::random_subset, 10, 5, 1e-10, false);

	// calculate the BIC
	int B = nb_gaussians * (1 + 2*nb_dimensions + nb_dimensions*(nb_dimensions-1)/2) - 1;
	double BIC = -2*gmm_model.sum_log_p(data) + log(nb_samples)*B;
	return std::make_pair(gmm_model, BIC);
}

std::deque<Eigen::MatrixXd> PointCloudToObstacle::cluster_surface_points(const Eigen::MatrixXd& surface_points)
{
	int nb_cluster_max = 10;
	Eigen::ArrayXd bic_values(nb_cluster_max);
	std::deque<arma::gmm_full> model_list(nb_cluster_max);

	// convert eigen to armadillo matrix using advanced initialization
	arma::mat data = arma::mat(surface_points.data(), surface_points.rows(), surface_points.cols());

	#pragma omp parallel for
	for(int i=0; i< nb_cluster_max; ++i)
	{
		auto models_and_bic = compute_bic(data, (i+1));
		model_list[i] = std::get<0>(models_and_bic);
		bic_values(i) = std::get<1>(models_and_bic);
	}

	// calculate the min of the bic
	int nb_clusters = bic_values.minCoeff() + 1;
	arma::gmm_full best_model = model_list[nb_clusters-1];

	// create the list of clusters
	std::deque<Eigen::MatrixXd> clusters(nb_clusters);

	#pragma omp parallel for
	for(int i=0; i<surface_points.cols(); ++i)
	{
		int cluster_id = best_model.assign(data.col(i), arma::eucl_dist);
		std::unique_lock<std::mutex> lock(this->mutex);
		clusters[cluster_id].conservativeResize(clusters[cluster_id].rows(), clusters[cluster_id].cols()+1);
		clusters[cluster_id].col(clusters[cluster_id].cols()-1) = surface_points.col(i);
		lock.unlock();
	}
	return clusters;
}

std::deque<Ellipsoid> PointCloudToObstacle::fit_ellipsoids(const Eigen::MatrixXd& surface_points, const int& nb_ellipsoids)
{
	arma::gmm_full gmm_model = learn_gmm_on_points(surface_points, nb_ellipsoids);
	std::deque<Ellipsoid> ellipsoid_list(nb_ellipsoids);

	// initialize the ellipsoid at the centroids
	for (int i = 0; i < nb_ellipsoids; ++i)
	{	
		arma::vec centroid = gmm_model.means.col(i);
		ellipsoid_list[i].set_position(centroid(0), centroid(1), centroid(2));
		// perform a svd on each covariance matrix
		arma::vec eigval;
		arma::mat eigvec;
		arma::mat cov = gmm_model.fcovs.slice(i);
		arma::eig_sym(eigval, eigvec, cov);
		
		// U contains the rotation
		Eigen::Matrix3d orientation = Eigen::Map<Eigen::Matrix3d>(eigvec.memptr(), eigvec.n_rows, eigvec.n_cols);
		// armadillo gives the values in ascending order so we need to reverse them
		orientation.rowwise().reverseInPlace();
		ellipsoid_list[i].set_orientation(Eigen::Quaterniond(orientation));
		
		// eigval contains the ellipse axis but in acending order
		Eigen::Array3d axis(eigval(2), eigval(1), eigval(0));

		// change the axis by 2*sqrt(2)*sqrt(axis)
		axis = 2 * sqrt(2) * axis.sqrt();
		ellipsoid_list[i].set_axis_lengths(axis);
	}
	return ellipsoid_list;
}

std::deque<Ellipsoid> PointCloudToObstacle::fit_ellipsoids(const Eigen::ArrayXd& distances, const double& starting_angle, const double& delta_angle, const double& z_coordinate, const int& nb_ellipsoids)
{

}