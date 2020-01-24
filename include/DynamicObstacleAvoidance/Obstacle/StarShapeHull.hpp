/**
 * @class StarShapeHull
 * @brief Class to define a star shape hull
 * @author Baptiste Busch
 * @date 2019/04/16
 */

#ifndef DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_STARSHAPEHULL_H_
#define DYNAMIC_OBSTACLE_AVOIDANCE_OBSTACLE_STARSHAPEHULL_H_

#include "DynamicObstacleAvoidance/Agent.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Obstacle.hpp"
#include "DynamicObstacleAvoidance/Obstacle/Ellipsoid.hpp"
#include "DynamicObstacleAvoidance/Utils/MathTools.hpp"
#include <algorithm>
#include <deque>
#define _USE_MATH_DEFINES 
#include <cmath>

#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/data.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/tools.hpp>
#include <limbo/tools/macros.hpp>
#include <limbo/serialize/text_archive.hpp>

namespace plt = matplotlibcpp;

using namespace limbo;

namespace DynamicObstacleAvoidance
{
	struct GPParams {
	    struct kernel_exp {
	        BO_PARAM(double, sigma_sq, 1.0);
	        BO_PARAM(double, l, 1.0);
	    };
	    struct kernel : public defaults::kernel {
	    	BO_PARAM(bool, optimize_noise, true);
	    	// BO_PARAM(double, noise, 0.2);
	    };
	    struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard {
	    };
	    struct opt_rprop : public defaults::opt_rprop {
	    	BO_PARAM(double, eps_stop, 1e-4);
	    	BO_PARAM(int, iterations, 20);
	    };
	};

	using Kernel_t = kernel::SquaredExpARD<GPParams>;
    using Mean_t = mean::Data<GPParams>;
    using GP_t = model::GP<GPParams, Kernel_t, Mean_t, model::gp::KernelLFOpt<GPParams>>;

	class StarShapeHull: public Obstacle
	{
	private:
		bool is_inside;
		unsigned int resolution;
		double min_radius;
		Eigen::MatrixXd cartesian_surface_points;
		Eigen::MatrixXd polar_surface_points;
		GP_t surface_regressor;

		Eigen::Vector3d compute_baricenter(const std::deque<std::shared_ptr<Obstacle> >& primitives);

		void initialize_regressor_parameters();

		std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd> > extract_regressor_data() const;

		void train_surface_regressor();

		Eigen::Vector3d predict_cartesian_point(double angle) const;

		Eigen::Vector3d predict_polar_point(double angle) const;

	public:
		explicit StarShapeHull(bool is_inside=false, unsigned int resolution=1000, double min_radius=0.5);

		explicit StarShapeHull(const std::deque<std::shared_ptr<Obstacle> >& primitives, bool is_inside=false, unsigned int resolution=1000, double min_radius=0.5);

		~StarShapeHull();

		double get_min_radius() const;

		Eigen::Vector3d compute_normal_to_agent(const Agent& agent) const;

		double compute_distance_to_point(const Eigen::Vector3d& point, double safety_margin=0.) const;

		void draw(const std::string& color="k") const;

		void set_resolution(unsigned int resolution);

		void compute_from_primitives(const std::deque<std::shared_ptr<Obstacle> >& primitives, Eigen::Vector3d reference_point);

		void compute_from_primitives(const std::deque<std::shared_ptr<Obstacle> >& primitives);

		bool point_is_inside(const Eigen::Vector3d& point) const;

		bool is_closed() const;
	};

	inline double StarShapeHull::get_min_radius() const
	{
		return this->min_radius;
	}

	inline void StarShapeHull::set_resolution(unsigned int resolution)
	{
		this->resolution = resolution;
		this->cartesian_surface_points = Eigen::MatrixXd::Zero(3, resolution);
		this->polar_surface_points = Eigen::MatrixXd::Zero(3, resolution);
	}
}
#endif