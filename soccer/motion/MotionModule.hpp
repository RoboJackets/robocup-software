// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QString>
#include <protobuf/RadioTx.pb.h>

#include <motion/Robot.hpp>
#include <framework/ConfigFile.hpp>

namespace Motion
{
	class MotionModule
	{
		public:
			MotionModule(SystemState *state, const ConfigFile::MotionModule& cfg);
			~MotionModule();

			void run();

		private:
			SystemState *_state;

			/** Robots **/
			Robot* _robots[5];
			
			const ConfigFile::MotionModule& _config;
	};
}
