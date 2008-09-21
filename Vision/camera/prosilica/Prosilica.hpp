#ifndef CAMERA__PROSOLICA_HPP
#define CAMERA__PROSOLICA_HPP

#include "../Base.h"
#include <PvApi.h>

#include <QMutex>
#include <QWaitCondition>

namespace Camera
{
	class Prosilica : public Base
	{
		public:
			/** create a new prosilica camera with the uid specified */
			Prosilica(unsigned int uid);
			virtual ~Prosilica();

			virtual void open();
			virtual void close();
			virtual bool is_open();

			virtual QWidget *configuration();
			virtual QSize size();
			virtual const Image *read_frame();

			/** initialize prosilica cameras for use */
			static void init();
			/** cleaup after using the prosilica library */
			static void destroy();

        private:
            static void callback(tPvFrame *frame);

			//true once the PvInitialize method has been called
			static bool _pvInitialized;

			Image _image;

			tPvCameraInfo _cameraInfo;
			tPvHandle _camera;

			//frame for image data
            static const int Num_Frames = 2;
			tPvFrame _frame[Num_Frames];

            // Protects the frames and current frame pointer.
            QMutex _mutex;
            QWaitCondition _frameAvailable;

            tPvFrame *_nextFrame;

			tPvUint32 _imageWidth;
			tPvUint32 _imageHeight;
	};
}

#endif /* CAMERA__PROSOLICA_HPP */
