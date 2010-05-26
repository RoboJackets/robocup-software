// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <LogFrame.hpp>

/** The system state is an aggregate data structure for storing current system
 * state information through a loop of the system. A state variable is not valid
 * until that module has run */
class SystemState : public Packet::LogFrame
{
};
