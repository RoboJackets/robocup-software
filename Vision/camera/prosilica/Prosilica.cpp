#include "Prosilica.hpp"

#include "../Camera_Thread.h"
#include "AttrCommand.h"
#include "AttrEnum.h"
#include "AttrFloat.h"
#include "AttrString.h"
#include "AttrUint32.h"

#include <stdexcept>
#include <unistd.h>
#include <dc1394/conversions.h>

#include <QTableWidget>
#include <QMutexLocker>

using namespace Camera;
using namespace std;

bool Prosilica::_pvInitialized = false;

Prosilica::Prosilica(unsigned int uid)
	: Base(), _image(640, 480, 3), _camera(0)
{
	//check to pv initialization
	if (!_pvInitialized)
	{
		Prosilica::init();
	}

	//check for camera and get its info
	if (ePvErrNotFound == PvCameraInfo(uid, &_cameraInfo))
	{
		throw runtime_error("Camera not found");
	}
}

Prosilica::~Prosilica()
{
	close();
}

void Prosilica::open()
{
	if (ePvErrSuccess != PvCameraOpen(_cameraInfo.UniqueId, ePvAccessMaster, &_camera))
	{
		throw runtime_error("Unable to open the camera");
	}

	if (PvAttrEnumSet(_camera, "PixelFormat", "Bayer8") != ePvErrSuccess)
	{
		throw runtime_error("Unable to set pixel format");
	}
	
    PvAttrUint32Set(_camera, "Width", _image.width());
	PvAttrUint32Set(_camera, "Height", _image.height());
	
	//start running frames
	if (ePvErrSuccess != PvCaptureStart(_camera))
	{
		throw runtime_error("Unable to start capture");
	}
	
    // Create and enqueue a frame
    unsigned long size = 0;
    PvAttrUint32Get(_camera, "TotalBytesPerFrame", &size);
//    size = 640 * 480 * 3;
    
    _nextFrame = 0;
    for (int i = 0; i < Num_Frames; ++i)
    {
        memset(&_frame[i], 0, sizeof(tPvFrame));
        _frame[i].ImageBuffer = malloc(size);
        _frame[i].ImageBufferSize = size;
        _frame[i].Context[0] = this;
        
        int ret = PvCaptureQueueFrame(_camera, &_frame[i], callback);
        if (ePvErrSuccess != ret)
        {
            printf("err %d\n", ret);
            throw runtime_error("Unable to queue frames");
        }
    }
    
    if (ePvErrSuccess != PvAttrEnumSet(_camera, "AcquisitionMode", "Continuous"))
	{
		throw runtime_error("Unable to set continuous acquisition mode");
	}
						    
	if (ePvErrSuccess != PvCommandRun(_camera, "AcquisitionStart"))
	{
		throw runtime_error("Unable to start Acquisition");
	}
}

void Prosilica::callback(tPvFrame *frame)
{
    Prosilica *cam = (Prosilica *)frame->Context[0];
    
    QMutexLocker ml(&cam->_mutex);
    
    cam->_nextFrame = frame;
    
    // Re-queue the frame
    if (ePvErrSuccess != PvCaptureQueueFrame(cam->_camera, frame, callback))
    {
        printf("Unable to queue frame %p\n", frame);
        return;
    }
    
    cam->_frameAvailable.wakeOne();
}

void Prosilica::close()
{
	//stop acquisition of frames
	PvCommandRun(_camera, "AcquisitionEnd");
	
	//stop frames
	PvCaptureEnd(_camera);
	
	PvCaptureQueueClear(_camera);
	
	PvCameraClose(_camera);
}

bool Prosilica::is_open()
{
	return _camera;
}

QWidget* Prosilica::configuration()
{
	QTableWidget* attrsTable = new QTableWidget();
	attrsTable->setColumnCount(2);
	
	tPvAttrListPtr attrs;
    unsigned long n = 0;
    if (PvAttrList(_camera, &attrs, &n))
    {
        printf("Can't get attribute list\n");
        return 0;
    }
    
    for (unsigned long i = 0; i < n; i++)
    {
        //addAttribute(attrs[i]);
    	const char* name = attrs[i];
    	
    	int row = attrsTable->rowCount();
    	attrsTable->insertRow(row);

	    QTableWidgetItem *label = new QTableWidgetItem(name);
	    label->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
	    attrsTable->setItem(row, 0, label);
	    
	    tPvAttributeInfo info;
	    if (!PvAttrInfo(_camera, name, &info))
	    {
	    	QWidget *w = 0;
    	    switch (info.Datatype)
    	    {
    	        case ePvDatatypeCommand:
    	            w = new AttrCommand(_camera, name);
    	            break;
    	        case ePvDatatypeString:
    	            w = new AttrString(_camera, name);
    	            break;
    	        case ePvDatatypeEnum:
    	            w = new AttrEnum(_camera, name);
    	            break;
    	        case ePvDatatypeUint32:
    	            w = new AttrUint32(_camera, name);
    	            break;
    	        case ePvDatatypeFloat32:
    	            w = new AttrFloat(_camera, name);
    	            break;
    	        default:
    	            printf("Attribute %s: unimplemented data type %d\n", name, info.Datatype);
    	            break;
    	    }

    	    if (!(info.Flags & ePvFlagWrite))
    	    {
    	        w->setEnabled(false);
    	    }
    	    
    	    attrsTable->setCellWidget(row, 1, w);
            
            AttrBase *attr = dynamic_cast<AttrBase *>(w);
            if (attr)
            {
                attr->reset();
            }
	    }
    }
	
	return attrsTable;
}

QSize Prosilica::size()
{
	return QSize(_image.width(), _image.height());
}

const Image* Prosilica::read_frame()
{
	if (!_camera)
	{
		printf("...\n");
		return &_image;
	}
	
    QMutexLocker ml(&_mutex);
    
    // If no frame is available, wait for one.
    if (!_nextFrame)
    {
        _frameAvailable.wait(&_mutex);
    }
    
	//if not successful, return the last frame
    if (_nextFrame->Status != ePvErrSuccess)
	{
		printf("no new image\n");
		return &_image;
	}

	QMutexLocker ml2(&_camera_thread->mutex);
    
    if (_nextFrame->Format == ePvFmtBgr24)
    {
        memcpy(_image.data(), _nextFrame->ImageBuffer, _nextFrame->Width * _nextFrame->Height * 3);
    } else if (_nextFrame->Format == ePvFmtMono8 || _nextFrame->Format == ePvFmtBayer8)
    {
#if 0
        // This is the same thing you get when the format is Bgr24.
        PvUtilityColorInterpolate(_nextFrame, (uint8_t*)_image.data() + 2, (uint8_t*)_image.data() + 1, (uint8_t*)_image.data(), 2, 0);
#else
        // This interpolator produces better edges than Prosilica's.
        dc1394_bayer_decoding_8bit((const uint8_t *)_nextFrame->ImageBuffer, (uint8_t *)_image.data(),
            _image.width(), _image.height(), DC1394_COLOR_FILTER_BGGR, DC1394_BAYER_METHOD_EDGESENSE);
#endif
    } else {
        printf("wrong format\n");
    }

	//printf("%x %x %x\n", ((uint8_t*)_image.data())[0], ((uint8_t*)_image.data())[1], ((uint8_t*)_image.data())[2]);
	//printf("%x %x %x\n", ((uint8_t*)_image.data())[32554], ((uint8_t*)_image.data())[32555], ((uint8_t*)_image.data())[32556]);
	
    // Indicate that this frame has been used.
    _nextFrame = 0;
    
    return &_image;
}

void Prosilica::init()
{
	if (ePvErrSuccess != PvInitialize())
	{
		throw runtime_error("Unable to initialize Pv Library");
	}

	//waiting is good...cameras take time to show up
	usleep(100 * 1000);
	_pvInitialized = true;
}

void Prosilica::destroy()
{
	PvUnInitialize();
}
