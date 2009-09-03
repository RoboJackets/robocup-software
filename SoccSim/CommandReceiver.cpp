#include "CommandReceiver.hpp"
#include "Physics/Env.hpp"

#include <Point.hpp>
#include <SimCommand.hpp>

#include <Network/Receiver.hpp>
#include <Network/Network.hpp>

CommandReceiver::CommandReceiver(Env *env):
    _env(env)
{
}

CommandReceiver::~CommandReceiver()
{
    terminate();
    wait();
}

void CommandReceiver::run()
{
    Network::Receiver receiver(Network::Address, Network::SimCommandPort);
    
    while (true)
    {
        Packet::SimCommand cmd;
        
        setTerminationEnabled(true);
        receiver.receive(cmd);
        setTerminationEnabled(false);
        
        _env->command(cmd);
    }
}
