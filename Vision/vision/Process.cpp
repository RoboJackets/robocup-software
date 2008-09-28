#include "Process.h"
#include "Colorseg.h"
#include "Spanner.h"
#include "Transform.h"
#include "Distortion.h"

#include <QMutexLocker>
#include <boost/foreach.hpp>

using namespace boost;
using namespace std;

Vision::Process::Process()
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

    //blue_id = new Offset3_ID(this, Blue, Vector_ID::Strive);

    //yellow_id = new Vector_ID(this, Yellow, Vector_ID::CMU);
    //yellow_id = new Offset3_ID(this, Yellow, Offset3_ID::ZJUNlict);
}

Vision::Process::~Process()
{
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
