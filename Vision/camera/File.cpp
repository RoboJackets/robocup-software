#include "File.h"
#include "File_Control.h"

extern "C"
{
#include <avformat.h>
#include <avcodec.h>
#include <swscale.h>
}

#include <stdexcept>

using namespace std;

bool Camera::File::_global_inited = false;

Camera::File::File(const QString &filename)
{
    if (!_global_inited)
    {
        av_register_all();
        _global_inited = true;
    }
    
    _filename = filename;
    
    format_ctx = 0;
    codec_ctx = 0;
    video_stream = -1;
    frame_in = 0;
    frame_rgb = 0;
    sws_ctx = 0;
    frame_period_us = 33333;
    ui = 0;
    
    gettimeofday(&last_frame_time, 0);
}

Camera::File::~File()
{
    close();
}

void Camera::File::open()
{
    QMutexLocker ml(&mutex);
    
    // http://www.inb.uni-luebeck.de/~boehme/using_libavcodec.html
    // see notes on recent changes
    
    QByteArray ascii = _filename.toAscii();
    const char *fn = ascii.data();
    printf("open \"%s\"\n", fn);
    
    if (av_open_input_file(&format_ctx, fn, 0, 0, 0))
    {
        throw runtime_error("av_open_input_file failed");
    }
    
    if (av_find_stream_info(format_ctx) < 0)
    {
        close();
        throw runtime_error("av_find_stream_info failed");
    }
    
    dump_format(format_ctx, 0, fn, 0);
    
    video_stream = -1;
    for (unsigned int i = 0; i < format_ctx->nb_streams; ++i)
    {
        if (format_ctx->streams[i]->codec->codec_type == CODEC_TYPE_VIDEO)
        {
            video_stream = i;
            break;
        }
    }
    
    if (video_stream == -1)
    {
        close();
        throw runtime_error("No video streams");
    }
    
    codec_ctx = format_ctx->streams[video_stream]->codec;
    AVCodec *codec = avcodec_find_decoder(codec_ctx->codec_id);
    if (!codec)
    {
        close();
        throw runtime_error("Can't find codec");
    }
    
    if (avcodec_open(codec_ctx, codec) < 0)
    {
        close();
        throw runtime_error("Can't open codec");
    }
    
    printf("CODEC: %s\n", codec->name);
    
    // Framerate
    frame_period_us = 1000000 * codec_ctx->time_base.num / codec_ctx->time_base.den;
    frame_period_us = 0;
    printf("Frame period: %dus\n", frame_period_us);
    
    // Set up the input frame
    frame_in = avcodec_alloc_frame();
    
    // Set up the RGB (output) frame
    _frame = Image(codec_ctx->width, codec_ctx->height, 4);
    
    frame_rgb = avcodec_alloc_frame();
    avpicture_fill((AVPicture *)frame_rgb, (uint8_t *)_frame.data(), PIX_FMT_RGB32, codec_ctx->width, codec_ctx->height);

    read_frame_from_file();
}

void Camera::File::close()
{
    QMutexLocker ml(&mutex);
    
    if (sws_ctx)
    {
        sws_freeContext(sws_ctx);
        sws_ctx = 0;
    }
    
    if (frame_in)
    {
        av_free(frame_in);
        frame_in = 0;
    }
    
    if (frame_rgb)
    {
        av_free(frame_rgb);
        frame_rgb = 0;
    }
    
    if (codec_ctx)
    {
        avcodec_close(codec_ctx);
        codec_ctx = 0;
    }
    
    if (format_ctx)
    {
        av_close_input_file(format_ctx);
        format_ctx = 0;
        video_stream = -1;
    }
}

bool Camera::File::is_open()
{
    return frame_rgb != 0;
}

QWidget *Camera::File::configuration()
{
    if (!ui)
    {
        ui = new File_Control(this);
    }
    
    return ui;
}

QSize Camera::File::size()
{
    if (codec_ctx)
    {
        return QSize(codec_ctx->width, codec_ctx->height);
    } else {
        return QSize();
    }
}

const Image *Camera::File::read_frame()
{
    QMutexLocker ml(&mutex);
    
    read_frame_from_file();
    
    // Wait for the remainder of the frame time
    struct timeval tv;
    gettimeofday(&tv, 0);
    
    // Don't calculate if more than one second elapsed, to avoid overflows
    // (we wouldn't sleep anyway)
    int elapsed_sec = tv.tv_sec - last_frame_time.tv_sec;
    if (elapsed_sec <= 1)
    {
        int elapsed_us = elapsed_sec * 1000000 + (tv.tv_usec - last_frame_time.tv_usec);
        if (elapsed_us < frame_period_us)
        {
            usleep(frame_period_us - elapsed_us);
        } else {
            usleep(1000);
        }
    }
    last_frame_time = tv;

    return &_frame;
}

void Camera::File::read_frame_from_file()
{
    AVPacket packet;
    
    // Read packets until the codec spits out a frame
    while (av_read_frame(format_ctx, &packet) >= 0)
    {
        if (packet.stream_index == video_stream)
        {
            int finished = 0;
            avcodec_decode_video(codec_ctx, frame_in, &finished, packet.data, packet.size);
            
            if (finished)
            {
                // Convert to RGB32
                int width = codec_ctx->width;
                int height = codec_ctx->height;
                if (!sws_ctx)
                {
                    sws_ctx = sws_getContext(width, height, codec_ctx->pix_fmt,
                                            width, height, PIX_FMT_RGB32,
                                            SWS_BICUBIC, 0, 0, 0);
                    if (!sws_ctx)
                    {
                        throw runtime_error("sws_getContext failed");
                    }
                }
                
                sws_scale(sws_ctx, frame_in->data, frame_in->linesize, 0, height,
                        frame_rgb->data, frame_rgb->linesize);
                
                break;
            }
        }
        
        //FIXME - Referee stream, when I figure out how to store it such that libavformat
        //  doesn't barf.
        
        av_free_packet(&packet);
    }
}
