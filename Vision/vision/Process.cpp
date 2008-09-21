#include "Process.h"
#include "Colorseg.h"
#include "Spanner.h"
#include "Transform.h"
#include "Distortion.h"
#include "Vector_ID.h"
#include "Offset3_ID.h"

#include <QMutexLocker>
#include <boost/foreach.hpp>

using namespace boost;
using namespace std;

Vision::Processor::~Processor()
{
}

////////

bool set_first = true;

Vision::Process::Process(Camera_Thread *camera_thread)
{
    send = set_first;
//    set_first = false;
	_camera_thread = camera_thread;

	distortion = new Distortion();
	ball_transform = new Transform();
	robot_transform = new Transform();

    colorseg = new Colorseg(camera_thread);
    processors.push_back(colorseg);

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

        processors.push_back(spanner[i]);
    }

    blue_id = new Offset3_ID(this, Blue, Vector_ID::Strive);
    processors.push_back(blue_id);

    yellow_id = new Vector_ID(this, Yellow, Vector_ID::CMU);
    //yellow_id = new Offset3_ID(this, Yellow, Offset3_ID::ZJUNlict);
    processors.push_back(yellow_id);
}

Vision::Process::~Process()
{
    BOOST_FOREACH(Processor *p, processors)
    {
        delete p;
    }

    delete distortion;
    delete ball_transform;
    delete robot_transform;
}

void Vision::Process::robot_radius(float r)
{
    QMutexLocker ml(&mutex);
    _robot_radius = r;
}

float Vision::Process::robot_radius() const
{
    QMutexLocker ml(&mutex);
    return _robot_radius;
}

void Vision::Process::run()
{
	// Set the frame size in the distortion parameters
	QSize size = _camera_thread->frame_size();
	distortion->frame_size(size.width(), size.height());

    BOOST_FOREACH(Processor *p, processors)
    {
        p->run();
    }
    
    //TODO make identifier (change span/groups to a sane robot structure)
    
    //send stuff out here!!
    list<Group*> yellows = spanner[Yellow]->groups();
    
    BOOST_FOREACH(Group *g, yellows)
    {
        printf("%d :: %f %f %f\n", g->id, g->center.x, g->center.y, g->direction);
        
        //TODO check direction_valid
    }
}
