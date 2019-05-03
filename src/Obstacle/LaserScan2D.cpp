#include "DynamicObstacleAvoidance/Obstacle/LaserScan2D.hpp"

LaserScan2D::LaserScan2D(const Eigen::ArrayXf& distances, const float& starting_angle, const float& delta_angle, const float& safety_margin, const float& z_coordinate)
{
	Pose reference_frame = compute_reference_frame(distances, starting_angle, delta_angle, safety_margin, z_coordinate);
	this->set_reference_position(reference_frame.get_position());
	this->regression = compute_regression(reference_frame, distances, starting_angle, delta_angle, z_coordinate);
}

LaserScan2D::~LaserScan2D()
{}


Polynomial LaserScan2D::compute_regression(const Pose& reference_frame, const Eigen::ArrayXf& distances, const float& starting_angle, const float& delta_angle, const float& z_coordinate) const
{

	std::vector<double> yz_coordinates(distances.size() * 2);
	std::vector<double> x_coordinates(distances.size());
	
	for(int i=0; i<distances.size(); ++i)
	{	
		// first express the points in the laser frame
		float theta = starting_angle + delta_angle * i;
		Eigen::Vector3f p(distances(i) * cos(theta), distances(i) * sin(theta), z_coordinate);

		// then tranform it in the reference frame
		Eigen::Vector3f p_ref = reference_frame.inverse() * p;

		// store the coordinates for regression
		x_coordinates[i] = p_ref(0);
		yz_coordinates[2*i] = p_ref(1);
		yz_coordinates[2*i+1] = p_ref(2);
	}
	return Polynomial(yz_coordinates, x_coordinates, 2, 5);
}

Pose LaserScan2D::compute_reference_frame(const Eigen::ArrayXf& distances, const float& starting_angle, const float& delta_angle, const float& safety_margin, const float& z_coordinate) const
{
	int size = distances.size();
	float theta = starting_angle + delta_angle * size / 2;
	float distance = distances(floor(size / 2)) + distances.maxCoeff() + safety_margin;
	Eigen::Vector3f reference_position(distance * cos(theta), distance * sin(theta), z_coordinate);

	float theta0 = starting_angle;
	float thetaN = starting_angle + delta_angle * size;
	float distance0 = distances(0);
	float distanceN = distances(size-1);
	Eigen::Vector3f p0(distance0 * cos(theta0), distance0 * sin(theta0), z_coordinate);
	Eigen::Vector3f pN(distanceN * cos(thetaN), distanceN * sin(thetaN), z_coordinate);

	// compute rotation matrix
	Eigen::Vector3f x_axis = -reference_position;
	Eigen::Vector3f y_axis = x_axis.cross(pN - p0);
	Eigen::Vector3f z_axis = x_axis.cross(y_axis);

	Eigen::Matrix3f rotation;
	rotation << x_axis(0), y_axis(0), z_axis(0),
	            x_axis(1), y_axis(1), z_axis(1),
	            x_axis(2), y_axis(2), z_axis(2);

	return Pose(reference_position, Eigen::Quaternionf(rotation));
}

Eigen::Vector3f LaserScan2D::compute_normal_to_external_point(const Eigen::Vector3f& external_point) const
{}

float LaserScan2D::compute_distance_to_external_point(const Eigen::Vector3f& external_point) const
{}