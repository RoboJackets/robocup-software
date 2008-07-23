#include "Tracker.h"
#include "../Camera_Thread.h"

#include <boost/foreach.hpp>

using namespace std;

Vision::Track::Track()
{
    used = false;
    age = 0;
    score = 0;
    report = -1;
}

////////

Vision::Tracker::Tracker(const std::list<Group *> &groups, int reports): _groups(groups)
{
    debug = false;
    need_ids = false;
    
    num_reports(reports);
}

Vision::Tracker::~Tracker()
{
}

void Vision::Tracker::num_reports(int n)
{
    reports.resize(n, 0);
}

void Vision::Tracker::run()
{
    BOOST_FOREACH(Track *track, tracks)
    {
        track->used = false;
    }
    
    list<Group *> working_groups = _groups;
    
    struct timeval cur_time;
    gettimeofday(&cur_time, 0);
    
    // Find the best group to continue each track
    BOOST_FOREACH(Track *track, tracks)
    {
        // Find the best continuation of this track
        
        // Linear prediction
        unsigned long delta_us = cur_time.tv_usec - track->last_time.tv_usec +
            (cur_time.tv_sec - track->last_time.tv_sec) * 1000000;
        Geometry::Point2d predict = track->group.report_center + track->velocity * delta_us / 1.0e6;
        track->report_pos = predict;
        
        Group *best_group = 0;
        float best_distance = -1;
        BOOST_FOREACH(Group *group, working_groups)
        {
            float distance_sq = (group->report_center - predict).magsq();
            
            // This group is the best continuation yet of this track if it is closer than
            // the last best, the ID is right, and the track has not already been found.
            if ((best_distance < 0 || distance_sq < best_distance) && group->id == track->group.id)
            {
                best_distance = distance_sq;
                best_group = group;
            }
        }
        
        if (best_group)
        {
            working_groups.remove(best_group);
            
            track->velocity = (best_group->report_center - track->group.report_center) /
                (delta_us / 1.0e6);
            track->last_time = cur_time;
        	
            track->group = *best_group;
            track->age = 0;
            track->used = true;
            track->score = best_distance;
            track->report_pos = best_group->report_center;
        } else {
            track->velocity = Geometry::Point2d();
        }
    }
    
    // Create new tracks from groups that weren't used to follow existing tracks
    BOOST_FOREACH(Group *group, working_groups)
    {
        Track *track = new Track();
        track->group = *group;
        track->used = true;
        track->report_pos = track->group.report_center;
        track->last_time = cur_time;
        tracks.push_back(track);
        
        if (debug)
        {
            printf("new track %d color %d at %.1f,%.1f\n",
                track->group.id, track->group.color,
                group->report_center.x, group->report_center.y);
        }
    }
    
    // Find or delete lost tracks
    list<Track *>::iterator i, next;
    for (i = tracks.begin(); i != tracks.end(); i = next)
    {
        next = i;
        ++next;
        Track *track = *i;
        if (!track->used)
        {
            track->age++;
            if (track->age > 10)
            {
                if (debug)
                {
                    printf("lost track %d color %d at %.1f,%.1f\n",
                        track->group.id, track->group.color,
                        track->group.report_center.x, track->group.report_center.y);
                }
                
                // Remove from reports
                for (unsigned int j = 0; j < reports.size(); ++j)
                {
                    if (reports[j] == track)
                    {
                        reports[j] = 0;
                    }
                }
                
                // Remove from tracks
                tracks.erase(i);
                
                delete track;
            }
        }
    }
    
    // Try to fill in missing reports
    for (unsigned int i = 0; i < reports.size(); ++i)
    {
        if (!reports[i])
        {
            // Find the best track for this report
            Track *best = 0;
            float best_score = 0;
            
            BOOST_FOREACH(Track *track, tracks)
            {
            	if (track->report >= 0)
            	{
            		// Already reported
            		continue;
            	}
            	
                if ((!best || track->score < best_score) && (!need_ids || track->group.id == (int)i))
                {
                    // Best one so far
                    best = track;
                    best_score = track->score;
                }
            }
            
            if (best)
            {
        		if (debug)
        		{
        			printf("Report %d is ID %d color %d\n",
                     i, best->group.id, best->group.color);
        		}
        		
                reports[i] = best;
                best->report = i;
            }
        }
    }
}
