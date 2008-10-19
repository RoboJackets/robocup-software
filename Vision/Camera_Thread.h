#ifndef _CAMERA_THREAD_H_
#define _CAMERA_THREAD_H_

#include <list>
#include <QThread>
#include <QMutex>
#include <QImage>
#include <QTime>

#include "vision/Process.h"

class Image;
class Image_Texture;

namespace Camera
{
	class Base;
}

/** The Camera Thread class grabs frames from the current camera
 * and passes them to windows and vision processing */
class Camera_Thread: public QThread
{
	Q_OBJECT;
		
	public:
		Camera_Thread();
		~Camera_Thread();

		static const std::list<Camera_Thread *> camera_threads()
		{
			return _camera_threads;
		}
		
		
		// Stops all camera threads
		static void stop_all();

		// Returns the current camera.  This may be NULL.
		Camera::Base *camera() const;

		// Sets the current camera.
		void camera(Camera::Base *cam);

		int update_time() const;
		void update_time(int ms);

		void pause(bool p);
		bool paused() const;

		void stop();

		// Copies the most recent from into <image>.
		// Thread safe.
		void get_frame(Image &image) const;

		QRgb get_pixel(int x, int y) const;

		// Updates a texture with the most recent frame.
		void frame_texture(Image_Texture &tex) const;

		// Returns the size of the most recent frame.
		QSize frame_size() const;

		// Returns a number that increases by one for each frame sent to vision processing.
		unsigned int frame_count() const;

		// Returns the number of milliseconds that elapsed between the last two frames.
		unsigned int frame_ms() const;

		// Returns the current frame.
		// ONLY FOR USE IN THE VISION THREAD.  Not thread safe.
		const Image *frame() const
		{
			return _frame;
		}
		
		Vision::Process* process;
		
		// Protects state for this thread so the GUI can change parameters.
		mutable QMutex mutex;

	Q_SIGNALS:
		void camera_changed(Camera::Base *camera);
		void new_frame();

	protected:
		// List of all camera threads
		static std::list<Camera_Thread *> _camera_threads;

		const Image *_frame;
		QSize _frame_size;
		unsigned int _frame_count;
		unsigned int _frame_ms;
		QTime _last_frame_time;
		volatile bool _can_redraw;

		// The current camera
		Camera::Base *_camera;

		// Minimum time in milliseconds between GUI updates
		int _update_time;

		// Set to false by stop()
		volatile bool _running;

		volatile bool _paused;

		// This handles the user event posted by the thread when a new frame is received.
		// It emits new_frame with the frame that was read.
		bool event(QEvent *e);

		// Entry point of the thread
		void run();
};

#endif // _CAMERA_THREAD_H_
