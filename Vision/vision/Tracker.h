#ifndef _VISION__TRACKER_H_
#define _VISION__TRACKER_H_

#include "Spanner.h"

#include <sys/time.h>

class Camera_Thread;

namespace Vision
{
    // A group, extended to include information needed for tracking
    class Track
    {
    public:
        Track();
        
        Group group;
        
        bool used;
        int age;
        
        // Report index or -1 if not in any report slot
        int report;
        
        // Used to choose tracks for reports.  Lower scores are better.
        float score;
        
        // Velocity in m/s.
        Geometry::Point2d velocity;
        
        // Reported position.  Either a prediction or an observation.
        Geometry::Point2d report_pos;
        
        // Time the track was last observed.
        struct timeval last_time;
    };
    
    class Tracker: public Processor
    {
    public:
        // Creates a tracker.  Groups are read from the given list of groups.
        // The reference to the list is expected never to change,
        // although the contents of the list will change from frame to frame.
        //
        // Sets the number of reports to <reports>.
        Tracker(const std::list<Group *> &groups, int reports);
        
        virtual ~Tracker();
        
        void num_reports(int n);
        
        // Fills in report_ids with (n) through (n + num_reports - 1).
        void start_id(int id);
        
        void run();
        
        // All groups being tracks
        std::list<Track *> tracks;
        
        // Tracks being reported downstream
        std::vector<Track *> reports;
        
        bool need_ids;
        bool debug;
        
    protected:
        const std::list<Group *> &_groups;
    };
}

#endif // _VISION__TRACKER_H_
