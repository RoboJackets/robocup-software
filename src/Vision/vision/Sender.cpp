#include "Sender.h"
#include "Process.h"
#include "../Camera_Thread.h"

#include <stdexcept>
#include <netinet/in.h>
#include <QSize>
#include <boost/format.hpp>
#include <Packet/IO.hpp>
#include <Packet/VisionData.hpp>

using namespace boost;
using namespace std;
using namespace Geometry;
using namespace Eigen;

// Switch sides
#if 1
Geometry::TransformMatrix global_transform = Geometry::TransformMatrix::rotate(180);
#else
Geometry::TransformMatrix global_transform = Geometry::TransformMatrix();
#endif

Vision::Sender::Sender()
{
	field_width(6.1);
	
	create_socket(yellow, CommonAddr, YellowBase + Packet::VisionData::Type);
	create_socket(blue, CommonAddr, BlueBase + Packet::VisionData::Type);
}

void Vision::Sender::create_socket(Team_Data &team, QHostAddress addr, uint16_t port)
{
	team.socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (team.socket < 0)
	{
		throw runtime_error(str(format("failed to create socket: %s") % strerror(errno)));
	}
	
	struct sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = 0;
	sin.sin_addr.s_addr = INADDR_ANY;
    if (bind(team.socket, (struct sockaddr *)&sin, sizeof(sin)))
	{
		throw runtime_error(str(format("bind failed: %s") % strerror(errno)));
	}
	
    team.sin.sin_family = AF_INET;
    team.sin.sin_port = htons(port);
    team.sin.sin_addr.s_addr = htonl(addr.toIPv4Address());
}

void Vision::Sender::field_width(float w)
{
	yellow.transform = TransformMatrix::rotate(90) * TransformMatrix::translate(Point2d(w / 2, 0));
	blue.transform = TransformMatrix::rotate(-90) * TransformMatrix::translate(Point2d(-w / 2, 0));
}

void Vision::Sender::update(Vision::Process *process)
{
	QMutexLocker ml(&mutex);
	
	// Update the ball
	update_object(_ball, process, process->ball_tracker->reports[0]);
	
	// Update robots
	for (int i = 0; i < 5; ++i)
	{
		update_object(yellow.robot[i], process, process->yellow_tracker->reports[i]);
		update_object(blue.robot[i], process, process->blue_tracker->reports[i]);
	}
	
	send_team(blue, yellow);
	send_team(yellow, blue);
}

void Vision::Sender::send_team(const Team_Data &self, const Team_Data &opp)
{
	Packet::VisionData data;
	
	if (_ball.owner)
	{
		data.ball.pos = self.transform * global_transform * _ball.track.report_pos;
        data.ball.vel = global_transform * self.transform.transformDirection(_ball.track.velocity);
		data.ball.valid = true;
	}
	
	for (int i = 0; i < 5; ++i)
	{
		if (self.robot[i].owner)
		{
			data.self[i].pos = self.transform * global_transform * self.robot[i].track.report_pos;
            data.self[i].vel = global_transform * self.transform.transformDirection(self.robot[i].track.velocity);
			data.self[i].theta = 180+self.transform.transformAngle(self.robot[i].track.group.direction * M_PI / 180.0) * 180.0 / M_PI;
            if (data.self[i].theta > 360)
                data.self[i].theta -= 360;
			data.self[i].valid = true;
		}

		if (opp.robot[i].owner)
		{
			data.opp[i].pos = self.transform * global_transform * opp.robot[i].track.report_pos;
            data.opp[i].vel = global_transform * self.transform.transformDirection(opp.robot[i].track.velocity);
			data.opp[i].theta = 180+self.transform.transformAngle(opp.robot[i].track.group.direction * M_PI / 180.0) * 180.0 / M_PI;
            if (data.opp[i].theta > 360)
                data.opp[i].theta -= 360;
			data.opp[i].valid = true;
		}
	}
	
	data.timeNow();
    
    sendto(self.socket, &data, sizeof(data), 0, (struct sockaddr *)&self.sin, sizeof(self.sin));
}

void Vision::Sender::update_object(Object_Data &object, Process *process, const Track *track)
{
#if 0
	// Ignore tracks from paused cameras
	if (process->camera_thread()->paused())
	{
		track = 0;
	}
#endif
	
	if (track)
	{
		// This process can report this object if it is already the owner, the object
		// has no owner, or this process has a track more recent than the last one reported.
		//
		// (This process will take ownership if the old process had a stale report *last*
		//  frame, even if it has a new report this frame, so we don't have to look up the
		//  corresponding object in all processes.)
		if (process == object.owner || !object.owner || track->age < object.track.age)
		{
			object.track = *track;
			object.owner = process;
		}
	} else {
		if (process == object.owner)
		{
			// Lost track, let another process take over
			object.owner = 0;
		}
	}
}
