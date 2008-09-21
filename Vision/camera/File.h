#ifndef _CAMERA__FILE_H_
#define _CAMERA__FILE_H_

#include "Base.h"

#include <QMutex>
#include <sys/time.h>

typedef struct AVFormatContext;
typedef struct AVCodecContext;
typedef struct AVFrame;
typedef struct SwsContext SwsContext;

namespace Camera
{
    class File_Control;
    
    class File: public Base
    {
        friend class File_Control;
        
    public:
        File(const QString &filename);
        ~File();
        
        virtual void open();
        virtual void close();
        virtual bool is_open();
        virtual QWidget *configuration();
        virtual QSize size();
        virtual const Image *read_frame();
        
        // Actually reads a frame from the file.
        // Does nothing if at EOF.
        // Call this to go to the next frame when paused.
        void read_frame_from_file();
        
    protected:
        QMutex mutex;
        
        QString _filename;
        Image _frame;
        
        // libavformat state
        AVFormatContext *format_ctx;
        AVCodecContext *codec_ctx;
        AVFrame *frame_in, *frame_rgb;
        struct ::SwsContext *sws_ctx;
        
        // Index of the video stream
        int video_stream;
        
        // Microseconds per frame
        int frame_period_us;
        
        struct timeval last_frame_time;
        
        File_Control *ui;
        
        // Set to true after global libavformat init is done
        static bool _global_inited;
    };
};

#endif // _CAMERA__FILE_H_
