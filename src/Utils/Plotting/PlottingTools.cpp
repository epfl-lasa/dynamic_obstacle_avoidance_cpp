#include "DynamicObstacleAvoidance/Utils/Plotting/PlottingTools.hpp"

namespace DynamicObstacleAvoidance
{
	void PlottingTools::plot_obstacles(const std::deque<std::shared_ptr<Obstacle> >& obstacles, const std::string& axis)
	{
		std::vector<std::string> colors = {"b", "g", "r", "c", "m", "y"};
		unsigned int i = 0;
		for(auto &o:obstacles)
		{
			o->draw(colors[i], axis);
			i = (i + 1) % colors.size();
		}
	}

	void PlottingTools::plot_agent(const Agent& agent, const std::string& axis, const std::string& color)
	{
		if (axis == "xy")
		{
			plt::plot({agent.get_position()(0)}, {agent.get_position()(1)}, color + "o");
		}
		else if (axis == "xz")
		{
			plt::plot({agent.get_position()(0)}, {agent.get_position()(2)}, color + "o");
		}
	}

	void PlottingTools::plot_agent_trajectory(const Eigen::Vector3d& goal, const std::deque<Eigen::Vector3d>& position_history, const std::string& axis)
	{
		std::vector<double> x(position_history.size());
		std::vector<double> y(position_history.size());
		std::vector<double> z(position_history.size());
		unsigned int i = 0;
		for(auto& point:position_history)
		{
			x.at(i) = point(0);
			y.at(i) = point(1);
			z.at(i) = point(2);
			++i;
		}
		if (axis == "xy")
		{
			plt::plot({goal(0)}, {goal(1)}, "rx");
			plt::plot(x, y, "k-");
		}
		else if (axis == "xz")
		{
			plt::plot({goal(0)}, {goal(2)}, "rx");
			plt::plot(x, z, "k-");
		}
	}

	void PlottingTools::plot_configuration(const Agent& agent, const std::deque<std::shared_ptr<Obstacle> >& obstacles, const Eigen::Vector3d& goal, const std::deque<Eigen::Vector3d>& position_history, const std::string& savefile, const bool& is_show)
	{
		plt::figure();
		plt::subplot(2, 1, 1);
		plot_obstacles(obstacles, "xy");
		plot_agent(agent, "xy");
		plot_agent_trajectory(goal, position_history, "xy");
		plt::xlim(-6, 6);
		plt::ylim(-6, 6);

		plt::subplot(2, 1, 2);
		plot_obstacles(obstacles, "xz");
		plot_agent(agent, "xy");
		plot_agent_trajectory(goal, position_history, "xz");
		plt::xlim(-6, 6);
		plt::ylim(-6, 6);

		if(savefile.compare(""))
		{
			std::string path = "/tmp/" + savefile + ".png";
			plt::save(path);
		}
		if(is_show)
		{
			plt::show();
		}
		plt::close();
	}

	void PlottingTools::plot_configuration(const std::deque<std::shared_ptr<Obstacle> >& obstacles, const std::string& savefile, const bool& is_show)
	{
		plt::figure();
		plt::subplot(2, 1, 1);
		plot_obstacles(obstacles, "xy");
		plt::xlim(-6, 6);
		plt::ylim(-6, 6);

		plt::subplot(2, 1, 2);
		plot_obstacles(obstacles, "xz");
		plt::xlim(-6, 6);
		plt::ylim(-6, 6);

		if(savefile.compare(""))
		{
			std::string path = "/tmp/" + savefile + ".png";
			plt::save(path);
		}
		if(is_show)
		{
			plt::show();
		}
		plt::close();
	}

	void PlottingTools::plot_guard_configuration(const Agent& platform, const Agent& guard, const std::deque<std::shared_ptr<Obstacle> >& obstacles, const Eigen::Vector3d& goal, const std::deque<Eigen::Vector3d>& position_history, const std::string& savefile, const bool& is_show)
	{
		plt::figure();
		plt::subplot(2, 1, 1);
		plot_obstacles(obstacles, "xy");
		plot_agent(platform, "xy");
		plot_agent(guard, "xy", "g");
		plot_agent_trajectory(goal, position_history, "xy");
		plt::xlim(-6, 6);
		plt::ylim(-6, 6);

		plt::subplot(2, 1, 2);
		plot_obstacles(obstacles, "xz");
		plot_agent(platform, "xz");
		plot_agent(guard, "xz", "g");
		plot_agent_trajectory(goal, position_history, "xz");
		plt::xlim(-6, 6);
		plt::ylim(-6, 6);

		if(savefile.compare(""))
		{
			std::string path = "/tmp/" + savefile + ".png";
			plt::save(path);
		}
		if(is_show)
		{
			plt::show();
		}
		plt::close();
	}
}