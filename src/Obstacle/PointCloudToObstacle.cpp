#include "DynamicObstacleAvoidance/Obstacle/PointCloudToObstacle.hpp"

PointCloudToObstacle::PointCloudToObstacle(const double& epsilon, const int& min_points_by_cluster, const double& safety_margin):
cluster_algorithm(epsilon, min_points_by_cluster), safety_margin(safety_margin)
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

std::deque<Eigen::MatrixXd> PointCloudToObstacle::cluster_surface_points(const Eigen::MatrixXd& surface_points)
{
	// cluster the surface points
	arma::mat data = arma::mat(surface_points.data(), surface_points.rows(), surface_points.cols());

	arma::Row<size_t> assignments;
	const size_t nb_clusters = this->cluster_algorithm.Cluster(data, assignments);

	// initialize the list of clusters
	std::deque<Eigen::MatrixXd> clusters(nb_clusters);
	#pragma omp parallel for
	for(int i=0; i<nb_clusters; ++i)
	{
		clusters[i].resize(surface_points.rows(), std::count(assignments.begin(), assignments.end(), i));
	}

	// fill all the clusters
	std::vector<int> nb_points_by_cluster(nb_clusters, 0);
	#pragma omp parallel for
	for(int i=0; i<surface_points.cols(); ++i)
	{
		if(assignments[i] != SIZE_MAX)
		{
			// get the cluster id assigned to th point
			int cluster_id = assignments[i];
			// put it in the cluster
			std::unique_lock<std::mutex> lock(this->mutex);
			clusters[cluster_id].col(nb_points_by_cluster[cluster_id]) = surface_points.col(i);
			++nb_points_by_cluster[cluster_id];
			lock.unlock();
		}
	}
	return clusters;
}

std::deque<std::unique_ptr<Ellipsoid> > PointCloudToObstacle::fit_ellipsoids_on_cluster(const Eigen::MatrixXd& surface_points, const int& nb_ellipsoids)
{
	arma::gmm_full gmm_model = learn_gmm_on_points(surface_points, nb_ellipsoids);
	std::deque<std::unique_ptr<Ellipsoid> > ellipsoid_list;

	// initialize the ellipsoid at the centroids
	for (int i = 0; i < nb_ellipsoids; ++i)
	{
		arma::vec centroid = gmm_model.means.col(i);
		auto ptrE = std::make_unique<Ellipsoid>(centroid(0), centroid(1), centroid(2), this->safety_margin);
		// perform a svd on each covariance matrix
		arma::vec eigval;
		arma::mat eigvec;
		arma::mat cov = gmm_model.fcovs.slice(i);
		arma::eig_sym(eigval, eigvec, cov);
		
		// U contains the rotation
		Eigen::Matrix3d orientation = Eigen::Map<Eigen::Matrix3d>(eigvec.memptr(), eigvec.n_rows, eigvec.n_cols);
		// armadillo gives the values in ascending order so we need to reverse them
		orientation.rowwise().reverseInPlace();
		ptrE->set_orientation(Eigen::Quaterniond(orientation));
		
		// eigval contains the ellipse axis but in acending order
		Eigen::Array3d axis(eigval(2), eigval(1), eigval(0));

		// change the axis by 2*sqrt(2)*sqrt(axis)
		axis = 2 * sqrt(2) * axis.sqrt();
		ptrE->set_axis_lengths(axis);
		ellipsoid_list.push_back(std::move(ptrE));
	}
	return ellipsoid_list;
}

std::deque<std::unique_ptr<Obstacle> > PointCloudToObstacle::fit_obstacles(const Eigen::MatrixXd& surface_points)
{
	std::deque<std::unique_ptr<Obstacle> > fitted_obstacles;

	// first get the list of clusters to be fitted
	std::deque<Eigen::MatrixXd> clusters = this->cluster_surface_points(surface_points);

	// for each cluster generate an ellipsoid fitting
	for(Eigen::MatrixXd points:clusters)
	{
		std::deque<std::unique_ptr<Ellipsoid> > ellipsoid_list = this->fit_ellipsoids_on_cluster(points, 2);
		// set the reference point of the two ellipsoid as the baricenter
		Eigen::Vector3d reference_position = (ellipsoid_list[0]->get_position() + ellipsoid_list[1]->get_position()) / 2.;
		ellipsoid_list[0]->set_reference_position(reference_position);
		ellipsoid_list[1]->set_reference_position(reference_position);
		for(auto &ptrE:ellipsoid_list)
		{
			fitted_obstacles.push_back(std::move(ptrE));
		}
	}
	return fitted_obstacles;
}