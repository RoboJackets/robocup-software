#pragma once

#include "../Image.h"

#include <QList>
#include <QString>
#include <QWidget>

// Each type of video source will subclass Camera.
// The constructor needs to add (this) to _cameras.
// The destructor needs to remove (this) from _cameras.
//
// Create a Camera object for each available video source.
//
// The constructor for each subclass should take whatever configuration is
// needed to open the device but should not open it.
// Call open() to open the device and close() to close it.
// A device may be opened and closed repeatedly.

// How to get frames:
//  Callback
//  Loop in subclass
//  Loop in superclass, blocking get_frame in subclass
//
// Must be able to block.
// Needs to be separate thread from GUI.
// Need to notify windows (emit signal) on new frame with signal rate limit.
// Need to provide every frame to vision.

class Camera_Thread;

namespace Camera
{
    class Base
    {
    public:
        Base()
        {
            _camera_thread = 0;
        }
        
        virtual ~Base() {}
        
        // Returns a string describing the device.
        // This string should be unique to the particular device, including
        // e,g. a filename or device ID
        QString name() const { return _name; }
        
        void name(const QString& name)
        {
            _name = name;
        }
	
        // Opens the device.
        // Throws an exception on error (which must descend from std::exception)
        virtual void open() = 0;
        
        // Closes the device.
        virtual void close() = 0;
        
        // Returns true if the device is open.
        // Before open() is first called, a Camera is never open.
        virtual bool is_open() = 0;
        
        // Returns a configuration panel.
        // The caller may hide it but may not destroy it.
        //
        // If this camera has no configuration or is not open, it may return NULL.
        // A camera may provide a configuration panel when it is not open.
        //
        // All GUI code needs to be thread safe with respect to the camera code
        // since read_frame is called from a thread other than the GUI thread.
        //
        // The returned widget must be valid until the camera is closed.  It may be
        // destroyed by the Camera when it is closed, so the caller needs to call
        // this again to check.
        virtual QWidget *configuration() = 0;
        
        // Returns the size of the frames read by this camera.
        // The size must not change while the camera is open.
        virtual QSize size() = 0;	
        
        // Reads one frame.
        // This function should return the last frame received by the hardware if
        // it has not already been returned, or wait until the next frame is available.
        //
        // The image returned must be in one of the RGB32 formats
        // (4 bytes per pixel: BB GG RR xx).
        //
        // Since QImage produces shallow copies, it is ok to return by
        // value here.  The caller should always use the const
        // version of QImage::bits() to avoid causing a deep copy.
        //
        // All camera code needs to be thread safe with respect to the GUI code
        // since read_frame is called from a thread other than the GUI thread.
        virtual const Image *read_frame() = 0;
        
        Camera_Thread *camera_thread() const { return _camera_thread; }
        void camera_thread(Camera_Thread *ct) { _camera_thread = ct; }
        
        // Returns the list of all Cameras
        //static const QList<Base *> &cameras() { return _cameras; }
    
    protected:
        //static QList<Base *> _cameras;

        Camera_Thread *_camera_thread;

	QString _name;
    };
}
