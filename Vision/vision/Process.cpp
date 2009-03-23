#include "Process.h"
#include "Colorseg.h"
#include "Spanner.h"
#include "Transform.h"
#include "Distortion.h"
#include "identification/Vector_ID.h"

#include <sys/time.h>
#include <QMutexLocker>
#include <boost/foreach.hpp>

using namespace boost;
using namespace std;
using namespace Vision;

unsigned int Process::_nextID = 0;

Process::Process() :
	_sender("226.0.0.1", 1337), _procID(_nextID++)
{
	distortion = new Distortion();
	ball_transform = new Transform();
	robot_transform = new Transform();

    colorseg = new Colorseg();

    for (int i = 0; i < Num_Colors; ++i)
    {
        spanner[i] = new Spanner(colorseg, i);
        spanner[i]->distortion = distortion;

        if (i == Orange)
        {
        	spanner[i]->transform = ball_transform;
        } else {
        	spanner[i]->transform = robot_transform;
        }
    }

    blueId = Identifier::load("../config/teams/RoboJackets.xml", this, Blue);
    //blue_id = new Offset3_ID(this, Blue, Vector_ID::Strive);
    //yellow_id = new Vector_ID(this, Yellow, Vector_ID::CMU);
    //yellow_id = new Offset3_ID(this, Yellow, Offset3_ID::ZJUNlict);
}

Process::~Process()
{
    delete distortion;
    delete ball_transform;
    delete robot_transform;
}

void Process::robot_radius(float r)
{
    QMutexLocker ml(&mutex);
    _robot_radius = r;
}

float Process::robot_radius() const
{
    QMutexLocker ml(&mutex);
    return _robot_radius;
}

//data is assumed to be cleaned before this method
void Process::proc(const Image* img)
{
	colorseg->run(img);
	
	//array of group lists from each spanner
	//this array is passed to the identifier to identify the robots
	std::list<Group*> groups[Num_Colors];
	
	for (int i = 0; i < Num_Colors; ++i)
    {
		spanner[i]->run();
		groups[i] = spanner[i]->groups();
	}
	
	_visionPacket = Packet::Vision();
	
	std::list<Group*> balls = groups[Orange];
	for (std::list<Group*>::iterator iter = balls.begin() ; iter != balls.end() ; ++iter)
	{
		Group* g = *iter;
		Packet::Vision::Ball ball;
		ball.pos.x = g->center.x;
		ball.pos.y = g->center.y;
		
		_visionPacket.balls.push_back(ball);
	}
	
	blueId->run();
	const std::vector<Identifier::Robot>& blue = blueId->robots();
	for (unsigned int i=0 ; i<blue.size() ; ++i)
	{
		const Identifier::Robot& r = blue[i];
		
		Packet::Vision::Robot rb;
		rb.shell = r.id;
		rb.angle = r.angle;
		rb.pos = Geometry::Point2d(r.x,r.y);
		
		_visionPacket.blue.push_back(rb);
	}
	
	yellowId->run();
	const std::vector<Identifier::Robot>& yellow = yellowId->robots();
	for (unsigned int i=0 ; i<yellow.size() ; ++i)
	{
		const Identifier::Robot& r = yellow[i];
		
		Packet::Vision::Robot rb;
		rb.shell = r.id;
		rb.angle = r.angle;
		rb.pos = Geometry::Point2d(r.x,r.y);
		
		_visionPacket.yellow.push_back(rb);
	}
	
	_visionPacket.timestamp = timestamp();
	_visionPacket.camera = _procID;
	
	_sender.send(_visionPacket);
}

uint64_t Process::timestamp() const
{
	struct timeval time;
	gettimeofday(&time, 0);
	
	return time.tv_sec * 1000000 + time.tv_usec;
}
