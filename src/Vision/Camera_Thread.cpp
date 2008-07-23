#include "Camera_Thread.h"
#include "Camera_Thread.moc"

#include "Image_Texture.h"
#include "camera/Base.h"
#include "vision/Process.h"
#include "Image.h"

#include <QMutexLocker>
#include <QTime>
#include <QEvent>
#include <QApplication>
#include <boost/foreach.hpp>
#include <sys/time.h>

using namespace std;
using namespace boost;

std::list<Camera_Thread *> Camera_Thread::_camera_threads;

////////

// This event is posted by the thread loop when a frame is received.
// It causes Camera_Thread::event (in the GUI thread) to emit new_frame.
class Frame_Event: public QEvent
{
public:
    static const QEvent::Type Type = QEvent::User;
    
    Frame_Event(): QEvent(Type)
    {
    }
};

////////

Camera_Thread::Camera_Thread()
{
    _camera = 0;
    _update_time = 33;
    _running = false;
    _paused = false;
    process = 0;
    _frame = 0;
    _frame_count = 0;
    _frame_ms = 0;
    _can_redraw = true;
    
    _camera_threads.push_back(this);
}

Camera_Thread::~Camera_Thread()
{
    _camera_threads.remove(this);
}

void Camera_Thread::stop_all()
{
    // Swap because we don't want the destructor to change the list
    // while we're iterating through it.
    list<Camera_Thread *> threads;
    threads.swap(_camera_threads);
    
    BOOST_FOREACH(Camera_Thread *thread, threads)
    {
        thread->stop();
    }
}

Camera::Base *Camera_Thread::camera() const
{
    QMutexLocker ml(&mutex);
    return _camera;
}

void Camera_Thread::camera(Camera::Base *cam)
{
    QMutexLocker ml(&mutex);
    _camera = cam;
    _camera->camera_thread(this);
    ml.unlock();
    const Image *new_frame = _camera->read_frame();
    ml.relock();
    _frame = new_frame;
    _frame_size = QSize(_frame->width(), _frame->height());
    emit camera_changed(_camera);
}

int Camera_Thread::update_time() const
{
    QMutexLocker ml(&mutex);
    return _update_time;
}

void Camera_Thread::update_time(int ms)
{
    QMutexLocker ml(&mutex);
    _update_time = ms;
}

bool Camera_Thread::paused() const
{
    QMutexLocker ml(&mutex);
    return _paused;
}

void Camera_Thread::pause(bool p)
{
    QMutexLocker ml(&mutex);
    _paused = p;
}

void Camera_Thread::stop()
{
    _running = false;
    wait();
    
    if (_camera)
    {
        _camera->close();
    }
}

void Camera_Thread::get_frame(Image &image) const
{
    QMutexLocker ml(&mutex);
    image = *_frame;
}

QRgb Camera_Thread::get_pixel(int x, int y) const
{
    QMutexLocker ml(&mutex);
    return _frame->pixel(x, y);
}

void Camera_Thread::frame_texture(Image_Texture &tex) const
{
    QMutexLocker ml(&mutex);
    if (_frame)
    {
    	tex.update(*_frame);
    }
}

QSize Camera_Thread::frame_size() const
{
    QMutexLocker ml(&mutex);
    return _frame_size;
}

unsigned int Camera_Thread::frame_count() const
{
    QMutexLocker ml(&mutex);
    return _frame_count;
}

unsigned int Camera_Thread::frame_ms() const
{
    QMutexLocker ml(&mutex);
    return _frame_ms;
}

bool Camera_Thread::event(QEvent *e)
{
    if (e->type() == Frame_Event::Type)
    {
        emit new_frame();
        
        _can_redraw = true;
        
        return true;
    } else {
        return QObject::event(e);
    }
}

unsigned int delta_us(const struct timeval &t0, const struct timeval &t1)
{
	return (t1.tv_sec - t0.tv_sec) * 1000000 + t1.tv_usec - t0.tv_usec;
}

void Camera_Thread::run()
{
    setTerminationEnabled(true);
    
    // The last time new_frame was emitted
    QTime last_update_time = QTime::currentTime();
    
    // If starting paused, get a frame first
    mutex.lock();
    if (_paused)
    {
        mutex.unlock();
        const Image *new_frame = _camera->read_frame();
        mutex.lock();
        _frame = new_frame;
        _frame_size = QSize(_frame->width(), _frame->height());
    }
    mutex.unlock();
    
    // Main loop
    _running = true;
    while (_running)
    {
        mutex.lock();
        
        if (!_camera)
        {
            // Wait for a camera to be set
            mutex.unlock();
            msleep(250);
            continue;
        }
        
        struct timeval t0;
        gettimeofday(&t0, 0);
        
        if (_paused)
        {
            // Send the same frame at 10Hz
            mutex.unlock();
            msleep(100);
            mutex.lock();
        } else {
            // Read a new frame.
            // Don't hold the mutex since this can take a long time.
            mutex.unlock();
            const Image *new_frame = _camera->read_frame();
            mutex.lock();
            
            _frame = new_frame;
            if (!_frame)
            {
                _frame_size = QSize();
                mutex.unlock();
                continue;
            }
	    
            _frame_size = QSize(_frame->width(), _frame->height());
        }

        struct timeval t1;
        gettimeofday(&t1, 0);
        
        // Frame statistics
        ++_frame_count;
        QTime now = QTime::currentTime();
        _frame_ms = _last_frame_time.msecsTo(now);
        _last_frame_time = now;
        
        int minimum_time = _update_time;
        mutex.unlock();
        
        // Call vision processing
        if (process)
        {
            process->run();
        }
        struct timeval t2;
        gettimeofday(&t2, 0);
        
//        printf("cap %d proc %d\n", delta_us(t0, t1), delta_us(t1, t2));
        
        // Update the GUI
        if (_can_redraw && last_update_time.msecsTo(now) >= minimum_time)
        {
            // Post an event to the GUI thread to notify about the new frame
            // (can't emit in this thread since it's not the GUI thread)
            _can_redraw = false;
            qApp->postEvent(this, new Frame_Event());
            msleep(1);
            
            last_update_time = now;
        }
    }
}
