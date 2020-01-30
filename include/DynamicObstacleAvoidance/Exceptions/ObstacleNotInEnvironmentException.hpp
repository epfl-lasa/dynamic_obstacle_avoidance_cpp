#ifndef DYNAMICOBSTACLEAVOIDANCE_EXCEPTIONS_OBSTACLENOTINENVIRONMENTEXCEPTION_H_
#define DYNAMICOBSTACLEAVOIDANCE_EXCEPTIONS_OBSTACLENOTINENVIRONMENT_H_

#include <iostream>
#include <exception>

namespace DynamicObstacleAvoidance
{
	namespace Exceptions
	{
		class ObstacleNotInEnvironmentException: public std::runtime_error
		{
		public:
			explicit ObstacleNotInEnvironmentException(const std::string& obstacle_name) :
			runtime_error("Obstacle " + obstacle_name + " is not in the environment")
			{};
		};
	}
}
#endif