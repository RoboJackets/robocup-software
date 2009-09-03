#pragma once

#include <dc1394/control.h>
#include <vector>
#include <boost/utility.hpp>

#include "../Base.h"

class DCam_Config;

namespace Camera
{
	class DCam: public Base, boost::noncopyable
	{
		public:
			/** Must be called before trying to use any DCams.
			 * This will populate the available cameras vector.
			 * It can be called again if any more cameras were connected. */
			static void scanCameras();

			/** Cleans up any resources that were allocated for DCams.
			 * Init will have this called on atexit (stdlib.h) so the
			 * user does not need to call this */
			static void destroy();

			static const std::vector<DCam*>& availableCameras() { return cameras; }

            DCam(uint64_t guid);
            ~DCam();

            virtual void open();
            virtual void close();
            virtual bool is_open();
            virtual QWidget *configuration();
            virtual QSize size();
            virtual const Image *read_frame();

			virtual void exposure(unsigned int level);

			void stop();

			/** Resets the 1394 Bus. <br \>
			 * NOTE: This will reset the entire bus and all devices on
			 * it. So use sparingly.
			 * From dc1394/control.h: "Its primary use is if a program
			 * shuts down uncleanly and needs to free leftover ISO
			 * channels or bandwidth. A bus reset will free those things
			 *  as a side effect."*/
			void reset1394Bus();

            dc1394camera_t *camera() const { return _camera; }

            // Returns the configuration window in its full type
            DCam_Config *dcam_config() const { return _config; }

		protected:
            // One-time global initialization
            static void init();

            static std::vector<DCam*> cameras;
            static dc1394_t *_dc1394;

            // Instance variables
		dc1394camera_t* _camera;

            bool _initialized;
            Image _image;
            DCam_Config *_config;
	};
}
