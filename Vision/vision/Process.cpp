#include "Process.h"
#include "Colorseg.h"
#include "Spanner.h"
#include "Distortion.h"
#include "Tracker.h"
#include "Sender.h"
#include "Vector_ID.h"
#include "Offset3_ID.h"

#include <QMutexLocker>
#include <boost/foreach.hpp>

using namespace boost;

Vision::Processor::~Processor()
{
}

////////

Vision::Sender *Vision::Process::sender = 0;
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
    
    yellow_id = new Vector_ID(this, Yellow, Vector_ID::GaTech);
    //yellow_id = new Offset3_ID(this, Yellow, Offset3_ID::ZJUNlict);
    processors.push_back(yellow_id);
    
    ball_tracker = new Tracker(spanner[Orange]->groups(), 1);
    processors.push_back(ball_tracker);
    
    yellow_tracker = new Tracker(spanner[Yellow]->groups(), 5);
    yellow_tracker->need_ids = true;
    processors.push_back(yellow_tracker);
    
    blue_tracker = new Tracker(spanner[Blue]->groups(), 5);
    blue_tracker->need_ids = true;
    processors.push_back(blue_tracker);
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
    
    if (sender && send)
    {
    	sender->update(this);
    }
}
