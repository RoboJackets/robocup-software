#ifndef _VISION__PROCESS_H_
#define _VISION__PROCESS_H_

#include "Colors.h"

#include <QMutex>
#include <list>

class Camera_Thread;

namespace Vision
{
    class Colorseg;
    class Spanner;
    class Dot_ID;
    class Vector_ID;
    class Tracker;
    class Distortion;
    class Transform;
    class Sender;
    
    class Processor
    {
    public:
        virtual ~Processor();
        
        virtual void run() = 0;
    };
    
    class Process
    {
    public:
        Process(Camera_Thread *camera_thread);
        ~Process();
        
        // The image may change size from one frame to the next as
        // different cameras are selected.
        //
        // The image format will always be an RGB32 format
        // (4 bytes per pixel: BB GG RR xx)
        //
        // This will be called from the camera thread, so it must
        // not do any GUI operations.
        // Use QApplication::postEvent() to make the GUI do things.
        void run();
        
        bool send;
        
        Camera_Thread *camera_thread() const { return _camera_thread; }
        
        void robot_radius(float r);
        float robot_radius() const;
        
        Colorseg *colorseg;
        Spanner *spanner[Num_Colors];
        Processor *blue_id, *yellow_id;
        Tracker *ball_tracker, *yellow_tracker, *blue_tracker;
        
        Distortion *distortion;
        Transform *ball_transform;
        Transform *robot_transform;
        
        static Sender *sender;
        
        std::list<Processor *> processors;
    
    protected:
        mutable QMutex mutex;
        
    	Camera_Thread *_camera_thread;
        float _robot_radius;
    };
};

#endif // _VISION__PROCESS_H_
