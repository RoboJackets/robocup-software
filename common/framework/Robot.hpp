#pragma once

#include "Obstacle.hpp"
#include "ConfigFile.hpp"

namespace Gameplay
{
	class Behavior;
}

namespace Framework
{
	// The generated class Packet::LogFrame::Robot inherits from this.
	// Add fields here which need to be associated with each robot but which should not be logged.
	class Robot
	{
		public:	
			ObstacleGroup obstacles;
			
			ConfigFile::Robot config;
	};
}
