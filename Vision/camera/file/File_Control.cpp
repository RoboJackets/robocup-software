#include "File_Control.h"
#include "File_Control.moc"
#include "File.h"

extern "C"
{
#include <avformat.h>
}

#include <QMutexLocker>

Camera::File_Control::File_Control(File *file, QWidget *parent): QWidget(parent)
{
    _file = file;
    
    ui.setupUi(this);
    
    connect(ui.restart, SIGNAL(clicked()), SLOT(restart()));
    connect(ui.play, SIGNAL(clicked()), SLOT(play()));
    connect(ui.step, SIGNAL(clicked()), SLOT(step()));
    connect(ui.time_slider, SIGNAL(sliderMoved(int)), SLOT(seek()));
    
    setup_slider();
}

void Camera::File_Control::restart()
{
    QMutexLocker ml(&_file->mutex);
    
    if (av_seek_frame(_file->format_ctx, _file->video_stream, 0, AVSEEK_FLAG_BACKWARD) < 0)
    {
        printf("Seek failed\n");
    }
}

void Camera::File_Control::step()
{
    QMutexLocker ml(&_file->mutex);
}

void Camera::File_Control::play()
{
}

void Camera::File_Control::seek()
{
    QMutexLocker ml(&_file->mutex);
    
    AVRational slider_scale;
    slider_scale.num = 1;
    slider_scale.den = 10;
    int64_t time = av_rescale_q(ui.time_slider->sliderPosition(), slider_scale, _file->codec_ctx->time_base);
    
    //FIXME - Only when necessary?
    int flags = AVSEEK_FLAG_BACKWARD;
    
    if (av_seek_frame(_file->format_ctx, _file->video_stream, time, flags) < 0)
    {
        printf("Seek failed\n");
    }
}

void Camera::File_Control::setup_slider()
{
    int64_t duration = _file->format_ctx->streams[_file->video_stream]->duration;
    AVRational slider_scale;
    slider_scale.num = 1;
    slider_scale.den = 10;
    int decs = (int)av_rescale_q(duration, _file->codec_ctx->time_base, slider_scale);
    ui.time_slider->setMaximum(decs);
}
