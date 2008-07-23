#include <stdio.h>
#include <signal.h>

#include <QMutex>
#include <QWaitCondition>
#include <Packet/Ref.hpp>
#include <Packet/PacketReceiver.hpp>
#include <Packet/PacketSender.hpp>
#include <boost/filesystem.hpp>

#include "Named_Matrix.hpp"
#include "Play.hpp"
#include "Role.hpp"
#include "Playbook.hpp"
#include "Predicate.hpp"
#include "tactics/Goalie.hpp"
#include "Robot.hpp"
#include "Predicates.hpp"

using namespace std;
using namespace Packet;
using namespace boost::filesystem;

Named_Matrix mirror_matrix("mirror");

Packet::PacketSender* sender;

bool new_vision = false;;

void shutdown(int sig)
{
    printf("Shutdown\n");

    // Send a blank MotionCmd to stop all movement
    Packet::SkillCmd cmd;
    sender->send(cmd);

    exit(1);
}

void vision_receiver(const VisionData* vd)
{
    vision_packet = *vd;
    new_vision = true;
}

void skill_status_receiver(const SkillStatus* ss)
{
	skill_status_packet = *ss;
}

void ref_receiver(const Ref* ref)
{
	setup = false;
	running = false;
	waiting = false;
	stopped = false;
	halt = false;
	
	kickoff = false;
	direct = false;
	indirect = false;
	penalty = false;
	
	our_action = ref->ourStart;
	
	winning = (ref->goalsSelf > ref->goalsOpp);
	losing = (ref->goalsSelf < ref->goalsOpp);
	
	//Now check what are we setting up to do
	switch (ref->start)
	{
		case Packet::Ref::None:
			break;
		case Packet::Ref::Kickoff:
			kickoff = true; 
			break;
		case Packet::Ref::Penalty:
			penalty = true;
			break;
		case Packet::Ref::Direct:
			direct = true;
			break;
		case Packet::Ref::Indirect:
			indirect = true;
			break;
	}
	
	switch (ref->state)
	{
		case Packet::Ref::Halt:
			halt = true;
			break;
		case Packet::Ref::Stop:
			stopped = true;
			break;
		case Packet::Ref::Setup:
			setup = true;	
			break;
		case Packet::Ref::OppStart:
			waiting = true;
			break;
		case Packet::Ref::Running:
			running = true;
			break;
	}
}

void usage(const char *prog)
{
    printf("Usage: %s [-b|-y] [-ng] [play directories]\n", prog);
    printf("    -b    Blue team\n");
    printf("    -y    Yellow team\n");
    printf("    -ng   No goalie\n");
}

int main(int argc, char *argv[])
{
    Playbook playbook;

    Team team = UnknownTeam;
    bool no_goalie = false;
    for (int i = 1; i < argc; ++i)
    {
        const char *arg = argv[i];
        if (arg[0] == 0)
        {
            // Empty argument, shouldn't happen
            continue;
        } else if (arg[0] == '-')
        {
            // Switch
            if (!strcmp(arg, "-b") && team == UnknownTeam)
            {
                team = Blue;
            } else if (!strcmp(arg, "-y") && team == UnknownTeam)
            {
                team = Yellow;
            } else if (!strcmp(arg, "-ng"))
            {
                no_goalie = true;
            } else {
                usage(argv[0]);
                return 1;
            }
        } else {
            // Play file or directory
            if (is_directory(arg))
            {
                playbook.load_dir(arg);
            } else {
                playbook.load(arg);
            }
        }
    }

    if (team == UnknownTeam)
    {
        usage(argv[0]);
        return 1;
    }

    if (playbook.plays().empty())
    {
        printf("No plays loaded!\n\n");
        usage(argv[0]);
        return 1;
    }

    printf("Loaded %d plays\n", (int)playbook.plays().size());

    if (!no_goalie)
    {
        playbook.goalie(new Tactics::Goalie());
    }

    offense.value = true;

    sender = new Packet::PacketSender(team);

    struct sigaction act;
    memset(&act, 0, sizeof(act));
    act.sa_handler = shutdown;
    sigaction(SIGINT, &act, 0);

    PacketReceiver receiver(team);
    receiver.addType(vision_receiver);
    receiver.addType(ref_receiver);
    receiver.addType(skill_status_receiver);

    // Last known X-coordinate of the ball
    float ball_x = 0;

    while (true)
    {
        // Receive packets from everybody
        receiver.receive();

        if (new_vision)
        {
			if (vision_packet.ball.valid)
			{
				ball_x = vision_packet.ball.pos.x;
			}

			if (playbook.setup())
			{
				// Set up matrices, etc. at the beginning of the play.
				if (ball_x < 0)
				{
					mirror_matrix = Geometry::TransformMatrix::mirrorX;
				} else {
					mirror_matrix = Geometry::TransformMatrix();
				}
			}

			// Run behaviors
			playbook.run();

			// Send the motion command
			sender->send(skill_packet);

			new_vision = false;
        }
    }

    return 0;
}
